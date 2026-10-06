# Security

Who can reach SCOPIO, how keys work, and what is **not** protected. Read this before you hand out a key or put the Pi on a new network.

## API keys

Every gateway route except `GET /api/v1/health` needs a key. Keys live on the Pi in `ros2_ws/secrets/api_keys.json`, as `{"name": "<48 hex characters>"}`. Give each client (the UI, an agent, a script) its own name, so you can revoke one without touching the others.

All commands run on the Pi, in `scopio/ros2_ws`.

Create a key, or replace the key of an existing name:

```bash
python3 scripts/generate_api_key.py <name>
```

List the names (never the values):

```bash
python3 scripts/generate_api_key.py --list
```

Revoke a key:

```bash
python3 scripts/generate_api_key.py --revoke <name>
```

How the gateway treats keys:

- It reloads the file whenever it changes. Creating or revoking a key needs no restart.
- It **fails closed**: with the file missing or empty, every keyed route answers 401, and `/health` reports `auth_configured: false`.
- It compares keys in constant time.
- The script makes the file readable by its owner only.

**Never commit `ros2_ws/secrets/` or `ros2_ws/.env`.** The `.gitignore` covers both. A key file that was ever committed lives on in git history even after it is deleted, so treat every key in it as leaked: revoke it and make a new one. This applies to this repo: its first commit contains `ros2_ws/secrets/api_keys.json` and `ros2_ws/.env`.

If a key leaks, rotate it: run `generate_api_key.py <name>` again, which replaces it, then give the new key to that client.

## What is exposed

| Port or service | Reachable from | Authentication |
|---|---|---|
| Gateway, `:8000` (HTTP and WebSocket) | the LAN | API key |
| Camera server, `:8081` | the Pi only (bound to 127.0.0.1) | none |
| ROS 2 / DDS | **see below** | none |

**The gateway speaks plain HTTP.** Keys travel in clear text on the LAN. For access from outside the lab network, put the Pi on a VPN or behind a TLS reverse proxy. Never port-forward 8000 to the internet.

**Prefer the header to `?api_key=`.** A key in the query string appears in the gateway's access log (`docker compose logs gateway`), which anyone with Docker access on the Pi can read. Use the query parameter only where headers are impossible: a browser `<img>` tag or a WebSocket.

**The camera server has no authentication.** It is safe only because it listens on loopback. Keep `CAM_HOST=127.0.0.1`.

**The ROS graph is probably reachable from the LAN.** The containers use host networking, and nothing in the configuration limits DDS discovery to the Pi (no `ROS_AUTOMATIC_DISCOVERY_RANGE`, `ROS_LOCALHOST_ONLY` or `ROS_DOMAIN_ID` is set). With ROS 2 Jazzy's defaults, any machine on the same subnet running ROS 2 can likely discover the nodes and call their services directly, bypassing the API keys. `scripts/smoke_test.sh` even assumes this works from the LAN. <!-- TODO: verify on the rig whether the graph is reachable from another machine, and decide whether to set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST for the scopio and gateway services. --> Until this is closed, treat the lab network as trusted.

## What is not protected

- **No range limits.** The nodes do not limit stage travel, AWG voltages or temperature setpoints. Every request within the instrument's own limits is carried out.
- **Every key is all-powerful.** There are no roles. Any key holder, human or AI agent, can move the stage, switch the laser, drive the galvo, change the temperature, reset an instrument to factory settings, or send raw SCPI.
- **Instrument nodes expose their whole driver class.** Any public method of `DG1022Z` or `TC10LAB` is callable, including `reset`, the instruments' own saved-state and network settings, and raw `command`/`query`.
- **The WebSocket can publish to any topic**, with any message type.

Hand keys only to people and programs you would trust at the bench. An AI agent with a key can do everything a person at the bench can do, including turning on the laser.

## Next

- [API.md](API.md#authentication): how clients send the key
- [CONFIGURATION.md](CONFIGURATION.md#compose-environment): the network settings
- [ARCHITECTURE.md](ARCHITECTURE.md): what runs where
