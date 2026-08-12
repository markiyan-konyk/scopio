"""API-key authentication for the gateway.

Keys live in a JSON file on the Pi (NOT in git):

    ros2_ws/secrets/api_keys.json      {"ui": "<48 hex chars>", "agent": "..."}

Generate/rotate keys with:  python3 ros2_ws/scripts/generate_api_key.py <name>

Clients authenticate every request with either:
  * header       X-API-Key: <key>          (preferred)
  * query param  ?api_key=<key>            (for browser <img>/WebSocket, which
                                            cannot set headers)

Notes:
  * Comparison is constant-time (hmac.compare_digest) against every stored key.
  * The file is hot-reloaded when its mtime changes -- add/revoke keys without
    restarting the gateway.
  * If the file is missing or empty, ALL authenticated routes are denied (fail
    closed) and /api/v1/health reports auth_configured=false so it's obvious.
"""

import hmac
import json
import os
import threading

from fastapi import HTTPException, Request

KEYS_FILE = os.environ.get("SCOPIO_API_KEYS_FILE", "/secrets/api_keys.json")


class KeyStore:
    def __init__(self, path=KEYS_FILE):
        self.path = path
        self._lock = threading.Lock()
        self._keys = {}
        self._mtime = None
        self._load()

    def _load(self):
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            self._keys, self._mtime = {}, None
            return
        if mtime == self._mtime:
            return
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._keys = {str(k): str(v) for k, v in data.items()}
            self._mtime = mtime
        except (OSError, ValueError):
            self._keys, self._mtime = {}, None

    @property
    def configured(self):
        with self._lock:
            self._load()
            return bool(self._keys)

    def check(self, presented):
        """Return the key's name if presented matches a stored key, else None."""
        if not presented:
            return None
        # compare_digest raises TypeError on non-ASCII str; keys are hex, so
        # anything unencodable is simply wrong -- a 401, not a 500.
        try:
            presented.encode("ascii")
        except UnicodeEncodeError:
            return None
        with self._lock:
            self._load()
            match = None
            for name, key in self._keys.items():
                # Compare against every key (constant-time each) -- no early exit
                # pattern that could leak which key prefix matched.
                if hmac.compare_digest(presented, key):
                    match = name
            return match


keystore = KeyStore()


def extract_key(request: Request):
    return request.headers.get("x-api-key") or request.query_params.get("api_key")


async def require_api_key(request: Request):
    """FastAPI dependency: 401 unless a valid API key is presented."""
    name = keystore.check(extract_key(request))
    if name is None:
        raise HTTPException(
            status_code=401,
            detail="Missing or invalid API key. Send it as an 'X-API-Key' header "
                   "or an 'api_key' query parameter. Keys are generated on the "
                   "microscope with ros2_ws/scripts/generate_api_key.py.",
        )
    request.state.api_key_name = name
    return name
