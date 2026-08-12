"""JSON <-> ROS 2 message conversion for the gateway.

Two directions:

  build_msg(cls, data)  -- JSON dict -> ROS message instance (for service
                           requests, published messages and action goals).
  msg_to_jsonable(msg)  -- ROS message -> plain JSON-safe python structure.

THE NaN RULE (important, documented in docs/API.md):
Several SCOPIO interfaces (SetCameraControls, CalibrationSet) use NaN as the
"leave this field unchanged" sentinel. JSON has no NaN, and a float field left
out of the request would otherwise be built as 0.0 -- which is not "unchanged",
it is a command to set that gain to zero. So on the way IN:

  * JSON null (or the string "nan") on a float/double field -> NaN
  * a float field ABSENT from a SERVICE REQUEST             -> NaN
  * a float field absent from an ACTION GOAL or a published
    message                                                 -> 0.0 (the default)

Services are the ones that take partial "set this subset" bodies, and EVERY
float field in the frozen contract belongs to such a service (SetCameraControls,
CalibrationSet, SetFramerate) -- everything where 0 is a meaningful value
(positions, ranges, step counts) is an integer. Action goals are complete
requests where an absent `settle_s` genuinely means "don't pause", so they keep
the plain default. That is why `nan_for_missing` is a per-call flag rather than
a property of the field type.

On the way OUT, NaN/inf become null (JSON-safe).
"""

import math
from collections import OrderedDict

from rosidl_runtime_py import message_to_ordereddict, set_message_fields
from rosidl_runtime_py.utilities import get_message

FLOAT_TYPES = ("float", "double", "float32", "float64")

# Topics whose messages are far too large to serialize as JSON over the
# generic WebSocket path. Video is served properly at /api/v1/stream.mjpg.
BULKY_TYPES = {
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/Image",
}


def normalize_msg_type(type_str):
    """Accept 'pkg/msg/Name' or 'pkg/Name' and return 'pkg/msg/Name'."""
    parts = type_str.split("/")
    if len(parts) == 2:
        return f"{parts[0]}/msg/{parts[1]}"
    return type_str


def _is_float_field(field_type):
    return field_type in FLOAT_TYPES


def _is_float_sequence(field_type):
    # e.g. "sequence<double>", "double[3]"
    for f in FLOAT_TYPES:
        if field_type.startswith(f"sequence<{f}") or field_type.startswith(f"{f}["):
            return True
    return False


def _nested_msg_type(field_type):
    """Return the message type string of a nested/sequence message field, else None."""
    inner = field_type
    if inner.startswith("sequence<"):
        inner = inner[len("sequence<"):].split(",")[0].rstrip(">")
    inner = inner.split("[")[0]
    return inner if "/" in inner else None


def _prepare(data, msg_cls, nan_for_missing=False):
    """Recursively apply the NaN rule to a JSON-derived dict."""
    if not isinstance(data, dict):
        return data
    out = {}
    fields = msg_cls.get_fields_and_field_types()
    for key, value in data.items():
        ftype = fields.get(key)
        if ftype is None:
            out[key] = value  # unknown field: let set_message_fields raise
            continue
        if _is_float_field(ftype):
            if value is None or (isinstance(value, str) and value.lower() == "nan"):
                out[key] = float("nan")
            else:
                out[key] = value
        elif _is_float_sequence(ftype) and isinstance(value, list):
            out[key] = [float("nan") if v is None else v for v in value]
        else:
            nested = _nested_msg_type(ftype)
            if nested is not None:
                nested_cls = get_message(normalize_msg_type(nested))
                if isinstance(value, list):
                    out[key] = [_prepare(v, nested_cls, nan_for_missing) for v in value]
                else:
                    out[key] = _prepare(value, nested_cls, nan_for_missing)
            else:
                out[key] = value
    if nan_for_missing:
        for key, ftype in fields.items():
            if key not in out and _is_float_field(ftype):
                out[key] = float("nan")
    return out


def build_msg(msg_cls, data, nan_for_missing=False):
    """Build a ROS message of type msg_cls from a JSON-derived dict.

    nan_for_missing=True fills every float field the body left out with NaN --
    used for service requests, where omitting a field means "leave it alone".
    It must run even for an EMPTY body: `POST camera/set_controls {}` has to be
    a no-op, not "set every colour gain to zero".

    Raises ValueError with a readable message on bad/unknown fields.
    """
    msg = msg_cls()
    if data or nan_for_missing:
        try:
            set_message_fields(msg, _prepare(data or {}, msg_cls, nan_for_missing))
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(str(exc)) from exc
    return msg


def _sanitize(value, byte_cap=65536):
    """Make a message_to_ordereddict result JSON-safe."""
    if isinstance(value, float):
        return None if (math.isnan(value) or math.isinf(value)) else value
    if isinstance(value, (bytes, bytearray)):
        return list(value[:byte_cap])
    if isinstance(value, (OrderedDict, dict)):
        return {k: _sanitize(v, byte_cap) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize(v, byte_cap) for v in value]
    if hasattr(value, "tolist"):  # array.array / numpy arrays
        return _sanitize(value.tolist(), byte_cap)
    return value


def msg_to_jsonable(msg):
    """ROS message -> JSON-safe dict (NaN/inf -> null, byte arrays -> lists)."""
    return _sanitize(message_to_ordereddict(msg))


def field_schema(msg_cls):
    """{field: type} description of a message class, for /interfaces."""
    return dict(msg_cls.get_fields_and_field_types())
