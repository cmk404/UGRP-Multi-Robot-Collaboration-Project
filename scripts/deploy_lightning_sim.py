#!/usr/bin/env python3
"""Deploy the UGRP MuJoCo simulator to a Lightning Studio after `lightning login`."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOKEN_FILE = ROOT / ".sim_bridge_token"
URL_FILE = ROOT / ".sim_remote_url"
PACKAGE = ROOT / "cloud" / "lightning"


def endpoint_url(endpoint) -> str | None:
    # SDK endpoint shapes changed over time; prefer explicit public/url fields.
    for key in ("url", "public_url", "endpoint", "host"):
        value = getattr(endpoint, key, None)
        if isinstance(value, str) and value.startswith(("http://", "https://")):
            return value.rstrip("/")
    blob = str(endpoint)
    for part in blob.replace("'", " ").replace('"', ' ').split():
        if part.startswith(("https://", "http://")):
            return part.rstrip(",)>]")
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="ugrp-sim")
    ap.add_argument("--machine", default="CPU", help="CPU, T4, L4, L40S ...")
    args = ap.parse_args()

    token = TOKEN_FILE.read_text().strip() if TOKEN_FILE.exists() else ""
    if not token:
        print("missing .sim_bridge_token", file=sys.stderr)
        return 2

    try:
        from lightning_sdk import Studio
    except ImportError:
        print("install lightning-sdk first", file=sys.stderr)
        return 2

    try:
        studio = Studio(args.name, create_ok=True)
        print(f"starting Studio {args.name} on {args.machine}...")
        studio.start(args.machine)
        studio.set_env({"UGRP_SIM_TOKEN": token, "UGRP_SIM_SEED": "11"}, partial=True)
        studio.upload_folder(str(PACKAGE), remote_path="ugrp-sim", progress_bar=True)
        out, code = studio.run_with_exit_code(
            "cd ugrp-sim && python -m pip install -q -r requirements.txt"
        )
        if code:
            print(out, file=sys.stderr)
            return code
        # Kill only a previous copy of this simulator inside this Studio.
        studio.run("pkill -f '[p]ython server.py --host 0.0.0.0 --port 7860' || true")
        studio.run_and_detach("cd ugrp-sim && bash start.sh", timeout=5)
        endpoints = studio.add_ports({"ugrp-sim": 7860})
        url = next((endpoint_url(e) for e in endpoints if endpoint_url(e)), None)
        if not url:
            endpoints = studio.list_ports()
            url = next((endpoint_url(e) for e in endpoints if endpoint_url(e)), None)
        if url:
            URL_FILE.write_text(url + "\n")
            URL_FILE.chmod(0o600)
            print(f"simulation endpoint: {url}")
            print(f"saved: {URL_FILE.name}")
        else:
            print("Studio is running, but the SDK did not expose a parseable endpoint URL.")
            print("Open the Studio port 7860 and save its public URL to .sim_remote_url.")
        return 0
    except Exception as exc:
        msg = str(exc)
        if "auth" in msg.lower() or "login" in msg.lower() or "credential" in msg.lower():
            print("Lightning authentication required: run `lightning login` first.", file=sys.stderr)
        else:
            print(f"Lightning deploy failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
