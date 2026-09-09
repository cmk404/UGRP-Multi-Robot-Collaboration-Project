#!/usr/bin/env python3
from __future__ import annotations
import json, subprocess, sys, time
from pathlib import Path
from urllib.request import Request, urlopen

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
from sim.worker_contract import REMOTE_WORKER_CONTRACT, worker_contract_compatible
from sim.bridge_client import bridge_headers

HEALTH='http://127.0.0.1:8091/health'
RECOVERY='ugrp-sim-gpu-recover.service'
POLL=1.0
RECOVERY_MISS_LIMIT=3   # 3 seconds before asking the Mac worker to recover.
RECOVERY_COOLDOWN=60.0  # Retry transient allocation/start failures once per minute.
PROVIDER_PROBE_INTERVAL=15.0
RECOVERY_ACTIVITY_MAX_AGE=75.0  # Do not wake the Mac worker without recent SIM use.

def systemctl(*args):
    return subprocess.run(['systemctl','--user',*args],capture_output=True,text=True)

def service_active(name: str) -> bool:
    return systemctl('is-active',name).stdout.strip() in {'active','activating'}

def recovery_active():
    return service_active(RECOVERY)

def configured_provider_names() -> set[str]:
    try:
        # Reload the private worker environment on every probe so an operator
        # can enable or disable Mac recovery without restarting the watchdog.
        cmd = (
            "set -a; "
            "[ ! -f .env.gpu ] || . ./.env.gpu; "
            "exec /usr/bin/python3 scripts/gpu_worker_recover.py --preflight"
        )
        proc=subprocess.run(
            ['/bin/bash','-lc',cmd],
            cwd=str(ROOT),capture_output=True,text=True,timeout=20,
        )
        if proc.returncode != 0:
            return set()
        obj=json.loads(proc.stdout.strip().splitlines()[-1])
        return {str(name).strip().lower() for name in (obj.get('configured') or []) if str(name).strip()}
    except (OSError, subprocess.TimeoutExpired, ValueError, IndexError):
        return set()

def recovery_available() -> bool:
    return bool(configured_provider_names())

def provider_is_allowed(provider: object, configured: set[str]) -> bool:
    return bool(provider) and str(provider).strip().lower() in configured

def worker_contract_is_allowed(contract: object) -> bool:
    return worker_contract_compatible(contract)

def bridge_health():
    try:
        with urlopen(HEALTH,timeout=2) as r:
            return json.loads(r.read().decode())
    except Exception:
        return {}

def set_remote_authority(enabled: bool) -> bool:
    try:
        payload=json.dumps({'enabled':bool(enabled)}).encode()
        req=Request('http://127.0.0.1:8091/remote/authority',data=payload,headers=bridge_headers(),method='POST')
        with urlopen(req,timeout=3) as r:
            obj=json.loads(r.read().decode())
        return bool(obj.get('ok')) and bool(obj.get('remote_authoritative')) == bool(enabled)
    except Exception as exc:
        print(f'remote authority update failed: {exc}',flush=True)
        return False

def should_start_recovery(
    misses: int, *, now: float, last_attempt: float, active: bool,
    available: bool = True, recent_activity: bool = True,
) -> bool:
    return (
        misses >= RECOVERY_MISS_LIMIT
        and not active
        and available
        and recent_activity
        and (last_attempt <= 0.0 or now-last_attempt >= RECOVERY_COOLDOWN)
    )

def main():
    misses=0
    last_recovery_attempt=0.0
    last_provider_probe=0.0
    provider_available=False
    configured_providers: set[str] = set()
    last_instance_id=None

    while True:
        health=bridge_health()
        remote=bool(health.get('remote_ws_connected'))
        authoritative=bool(health.get('remote_authoritative'))
        inflight=health.get('inflight')
        instance_id=health.get('remote_instance_id')
        now=time.monotonic()

        if now-last_provider_probe >= PROVIDER_PROBE_INTERVAL:
            configured_providers=configured_provider_names()
            provider_available=bool(configured_providers)
            last_provider_probe=now

        remote_provider=health.get('remote_provider')
        provider_allowed=provider_is_allowed(remote_provider, configured_providers)
        contract_allowed=worker_contract_is_allowed(health.get('remote_worker_contract'))
        if remote and not (provider_allowed and contract_allowed):
            if authoritative and inflight is None:
                if set_remote_authority(False):
                    authoritative=False
                    print(f'rejected incompatible worker provider={remote_provider!r} contract={health.get("remote_worker_contract")!r}; allowed_providers={sorted(configured_providers)} expected_contract={REMOTE_WORKER_CONTRACT}', flush=True)
            # Keep the unexpected socket connected but treat it as unavailable.
            # A newly launched allowed worker will replace it at the bridge.
            remote=False

        if remote:
            misses=0
            last_recovery_attempt=0.0
            if not authoritative and inflight is None:
                # Never restart the coworker process here: doing so kills any
                # user request that is waiting for GPU recovery. Bridge/executive
                # episode IDs now separate same-process reconnects from a fresh
                # MuJoCo process and reset planner state without process death.
                if instance_id and instance_id != last_instance_id:
                    print(f'new Mac worker {instance_id}: episode boundary handled in bridge/executive', flush=True)
                if set_remote_authority(True):
                    last_instance_id=instance_id or last_instance_id
                    print('Mac worker authoritative', flush=True)
                else:
                    print('GPU authority grant failed; recovery will retry', flush=True)
            time.sleep(POLL)
            continue

        misses += 1
        activity_age = float(health.get('user_activity_age_s') or 1e9)
        if should_start_recovery(
            misses,
            now=now,
            last_attempt=last_recovery_attempt,
            active=recovery_active(),
            available=provider_available,
            recent_activity=activity_age <= RECOVERY_ACTIVITY_MAX_AGE,
        ):
            print('Mac worker absent: requesting recovery', flush=True)
            result=systemctl('start','--no-block',RECOVERY)
            last_recovery_attempt=now
            if result.returncode != 0:
                print(f'Mac worker recovery service start failed: {result.stderr.strip()}', flush=True)
        time.sleep(POLL)

if __name__=='__main__': main()
