"""GET /api/v1/interfaces -- self-describing discovery of the live graph.

Lists every service, topic and action currently visible (standard ROS plumbing
filtered out) with per-field schemas, so a developer -- or an agent -- can ask
the microscope what it can do instead of reading the docs. New nodes (e.g. the
planned temperature sensor/heater) show up here automatically.
"""

from rosidl_runtime_py.utilities import get_action, get_message, get_service

from .conversion import BULKY_TYPES, field_schema, normalize_msg_type

# Per-node ROS plumbing nobody calls over the gateway.
_NOISE_SERVICE_TYPES = (
    "rcl_interfaces/",
    "type_description_interfaces/",
)
_NOISE_TOPICS = {"/rosout", "/parameter_events"}


def interfaces_payload(bridge):
    services, topics, actions = bridge.tables()

    out = {"namespace": "/scopio", "services": {}, "topics": {}, "actions": {}}

    # Actions appear in the service/topic tables as _action/* internals; the
    # action table already names them properly, so drop the internals.
    def _is_action_internal(name):
        return "/_action/" in name

    for name, type_str in sorted(services.items()):
        if _is_action_internal(name):
            continue
        if any(type_str.startswith(p) for p in _NOISE_SERVICE_TYPES):
            continue
        try:
            srv = get_service(type_str)
        except Exception:
            continue
        out["services"][name] = {
            "type": type_str,
            "request": field_schema(srv.Request),
            "response": field_schema(srv.Response),
        }

    for name, type_str in sorted(topics.items()):
        if name in _NOISE_TOPICS or _is_action_internal(name):
            continue
        try:
            msg = get_message(normalize_msg_type(type_str))
        except Exception:
            continue
        out["topics"][name] = {
            "type": type_str,
            "fields": field_schema(msg),
            "subscribable_over_ws": normalize_msg_type(type_str) not in BULKY_TYPES,
        }

    for name, type_str in sorted(actions.items()):
        try:
            act = get_action(type_str)
        except Exception:
            continue
        out["actions"][name] = {
            "type": type_str,
            "goal": field_schema(act.Goal),
            "result": field_schema(act.Result),
            "feedback": field_schema(act.Feedback),
        }

    return out
