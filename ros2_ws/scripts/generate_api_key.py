#!/usr/bin/env python3
"""Generate (or rotate) an API key for the SCOPIO gateway.

Run ON THE MICROSCOPE (the Raspberry Pi):

    python3 ros2_ws/scripts/generate_api_key.py <name>      # e.g. ui, agent
    python3 ros2_ws/scripts/generate_api_key.py --list
    python3 ros2_ws/scripts/generate_api_key.py --revoke <name>

Keys are stored in ros2_ws/secrets/api_keys.json (gitignored, chmod 600).
The gateway hot-reloads the file -- no restart needed. Give the printed key
to the client app (SCOPIO_API_KEY env var / X-API-Key header).
"""

import argparse
import json
import os
import secrets
import stat
import sys

SECRETS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "secrets")
KEYS_FILE = os.environ.get("SCOPIO_API_KEYS_FILE",
                           os.path.join(SECRETS_DIR, "api_keys.json"))


def load():
    try:
        with open(KEYS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save(keys):
    os.makedirs(os.path.dirname(KEYS_FILE), exist_ok=True)
    with open(KEYS_FILE, "w", encoding="utf-8") as f:
        json.dump(keys, f, indent=2)
        f.write("\n")
    try:  # best effort on Windows
        os.chmod(KEYS_FILE, stat.S_IRUSR | stat.S_IWUSR)
    except OSError:
        pass


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("name", nargs="?", help="key name to create/rotate (e.g. ui)")
    p.add_argument("--list", action="store_true", help="list key names (not values)")
    p.add_argument("--revoke", metavar="NAME", help="delete a key")
    args = p.parse_args()

    keys = load()

    if args.list:
        for name in sorted(keys):
            print(name)
        return
    if args.revoke:
        if keys.pop(args.revoke, None) is None:
            sys.exit(f"No key named '{args.revoke}' in {KEYS_FILE}")
        save(keys)
        print(f"Revoked '{args.revoke}'. The gateway picks this up automatically.")
        return
    if not args.name:
        p.print_help()
        sys.exit(2)

    action = "Rotated" if args.name in keys else "Created"
    keys[args.name] = secrets.token_hex(24)
    save(keys)
    print(f"{action} key '{args.name}' in {KEYS_FILE}:")
    print()
    print(f"    {keys[args.name]}")
    print()
    print("Give this to the client app:  SCOPIO_API_KEY=<key>  "
          "(sent as the X-API-Key header)")


if __name__ == "__main__":
    main()
