"""Transport abstractions for MasterPi dashboard control execution."""

from __future__ import annotations

import abc
import ipaddress
import os
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Callable, Sequence

DEFAULT_TARGET = "ugrp1"
DEFAULT_PI_SCRIPT = "/home/ugrp1/MasterPi/tools/masterpi_control.py"
DEFAULT_PYTHON_BIN = "python3"
DEFAULT_SSH_TIMEOUT = 2
DEFAULT_SSH_CONTROL_PATH = os.path.expanduser("~/.ssh/masterpi-control-%C")
MIN_EFFECTIVE_WHEEL_SPEED = 31
MAX_CHASSIS_SPEED = 40



def validate_chassis_speed(speed: int) -> int:
    if isinstance(speed, bool) or not isinstance(speed, int):
        raise ValueError("drive speed must be an integer")
    if not MIN_EFFECTIVE_WHEEL_SPEED <= speed <= MAX_CHASSIS_SPEED:
        raise ValueError(
            f"drive speed must be {MIN_EFFECTIVE_WHEEL_SPEED}..{MAX_CHASSIS_SPEED}; "
            "values <=30 do not move this chassis"
        )
    return speed


def is_ipv4(addr: str) -> bool:
    try:
        return ipaddress.IPv4Address(addr).version == 4
    except ValueError:
        return False


def default_get_local_ips() -> set[str]:
    ips = {"127.0.0.1"}
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        if local_ip:
            ips.add(local_ip)
    except Exception:
        pass
    return ips


def default_resolve_mdns(host: str = "ugrp1.local") -> str | None:
    command = (
        ["dscacheutil", "-q", "host", "-a", "name", host]
        if sys.platform == "darwin"
        else ["getent", "ahostsv4", host]
    )
    try:
        result = subprocess.run(
            command,
            shell=False,
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        for line in result.stdout.splitlines():
            parts = line.replace("ip_address:", "").split()
            for part in parts:
                if is_ipv4(part):
                    return part
    except Exception:
        pass
    return None


def default_probe_port(ip: str, port: int = 22, timeout: float = 0.2) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except Exception:
        return False


class AddressResolver:
    """Bounded resolver evaluating candidates in strict precedence order."""

    def __init__(
        self,
        *,
        mdns_resolver: Callable[[str], str | None] | None = None,
        port_prober: Callable[[str, int, float], bool] | None = None,
        local_ip_getter: Callable[[], set[str]] | None = None,
        scan_workers: int = 30,
        probe_timeout: float = 0.15,
    ) -> None:
        self.mdns_resolver = mdns_resolver or default_resolve_mdns
        self.port_prober = port_prober or default_probe_port
        self.local_ip_getter = local_ip_getter or default_get_local_ips
        self.scan_workers = scan_workers
        self.probe_timeout = probe_timeout

    def scan_subnet(self, local_ip: str, exclude_ips: set[str]) -> list[str]:
        try:
            network = ipaddress.IPv4Network(f"{local_ip}/24", strict=False)
        except ValueError:
            return []

        hosts_to_scan = [str(h) for h in network.hosts() if str(h) not in exclude_ips]
        candidates: list[str] = []

        with ThreadPoolExecutor(max_workers=min(self.scan_workers, len(hosts_to_scan) or 1)) as pool:
            future_to_ip = {
                pool.submit(self.port_prober, ip, 22, self.probe_timeout): ip
                for ip in hosts_to_scan
            }
            for future in as_completed(future_to_ip):
                ip = future_to_ip[future]
                try:
                    if future.result():
                        candidates.append(ip)
                except Exception:
                    pass
        return sorted(candidates)

    def find_candidates(
        self,
        *,
        explicit_target: str | None = None,
        cached_address: str | None = None,
    ) -> list[str]:
        candidates: list[str] = []
        seen: set[str] = set()

        def add(c: str | None) -> None:
            if c and c not in seen:
                seen.add(c)
                candidates.append(c)

        # 1. Explicit target (if provided and different from default alias or is an IP)
        if explicit_target and (explicit_target != DEFAULT_TARGET or is_ipv4(explicit_target)):
            add(explicit_target)

        # 2. Cached address
        if cached_address:
            add(cached_address)

        # 3. Bounded ugrp1.local mDNS lookup
        mdns_ip = self.mdns_resolver("ugrp1.local")
        if mdns_ip:
            add(mdns_ip)

        # 4. Local subnet /24 scan (excluding local interface IPs)
        local_ips = self.local_ip_getter()
        for lip in list(local_ips):
            if lip != "127.0.0.1" and is_ipv4(lip):
                for scanned_ip in self.scan_subnet(lip, local_ips):
                    add(scanned_ip)

        return candidates


class ControlTransport(abc.ABC):
    """Abstract base for command generation across different execution targets."""

    @property
    @abc.abstractmethod
    def target_name(self) -> str:
        """Name of the target for reporting in /api/status."""
        ...

    @property
    def cached_address(self) -> str | None:
        return None

    def invalidate_cache(self) -> None:
        pass

    @abc.abstractmethod
    def status_argv(self) -> list[str]:
        """Command to check connectivity/readiness without actuating motors."""
        ...

    @abc.abstractmethod
    def probe_argv(self) -> list[str]:
        """Command to probe hardware and read battery/controller status."""
        ...

    @abc.abstractmethod
    def stop_argv(self) -> list[str]:
        """Command to stop all motors."""
        ...

    @abc.abstractmethod
    def drive_argv(self, direction: str, speed: int, duration: float) -> list[str]:
        """Command to run a bounded chassis drive action."""
        ...

    @abc.abstractmethod
    def servo_argv(self, servo: int, pulse: int, duration: float) -> list[str]:
        """Command to actuate a single PWM servo."""
        ...


class SSHTransport(ControlTransport):
    """Remote execution over SSH."""

    def __init__(
        self,
        target: str = DEFAULT_TARGET,
        script_path: str = DEFAULT_PI_SCRIPT,
        *,
        python_bin: str = DEFAULT_PYTHON_BIN,
        connect_timeout: int = DEFAULT_SSH_TIMEOUT,
        dry_run: bool = False,
        extra_ssh_opts: Sequence[str] | None = None,
        cached_address: str | None = None,
        resolver: AddressResolver | None = None,
    ) -> None:
        self.target = target
        self.script_path = script_path
        self.python_bin = python_bin
        self.connect_timeout = connect_timeout
        self.dry_run = dry_run
        self.extra_ssh_opts = list(extra_ssh_opts or [])
        self._cached_address: str | None = cached_address
        self.resolver = resolver or AddressResolver()

    @property
    def target_name(self) -> str:
        return self.target

    @property
    def cached_address(self) -> str | None:
        return self._cached_address

    def invalidate_cache(self) -> None:
        self._cached_address = None

    def _ssh_base(self, address: str | None = None) -> list[str]:
        addr = address if address is not None else self._cached_address
        opts = [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            f"ConnectTimeout={self.connect_timeout}",
            "-o",
            "ConnectionAttempts=1",
            "-o",
            "ServerAliveInterval=1",
            "-o",
            "ServerAliveCountMax=1",
            "-o",
            "ControlMaster=auto",
            "-o",
            "ControlPersist=120",
            "-o",
            f"ControlPath={DEFAULT_SSH_CONTROL_PATH}",
        ]
        if addr and addr != self.target:
            opts.extend([
                "-o",
                f"HostName={addr}",
                "-o",
                "HostKeyAlias=ugrp1.local",
            ])
        opts.extend(self.extra_ssh_opts)
        opts.append(self.target)
        return opts

    def auth_check_argv(self, address: str | None = None) -> list[str]:
        return self._ssh_base(address=address) + ["true"]

    def status_argv(self) -> list[str]:
        return self._ssh_base() + ["true"]

    def probe_argv(self) -> list[str]:
        return self._ssh_base() + [self.python_bin, self.script_path, "probe"]

    def stop_argv(self) -> list[str]:
        cmd = [self.python_bin, self.script_path, "stop"]
        if self.dry_run:
            cmd.append("--dry-run")
        return self._ssh_base() + cmd

    def drive_argv(self, direction: str, speed: int, duration: float) -> list[str]:
        speed = validate_chassis_speed(speed)
        cmd = [
            self.python_bin,
            self.script_path,
            "drive",
            direction,
            "--speed",
            str(speed),
            "--duration",
            format(duration, ".6g"),
        ]
        if self.dry_run:
            cmd.append("--dry-run")
        return self._ssh_base() + cmd

    def servo_argv(self, servo: int, pulse: int, duration: float) -> list[str]:
        cmd = [
            self.python_bin,
            self.script_path,
            "servo",
            str(servo),
            str(pulse),
            "--duration",
            format(duration, ".6g"),
        ]
        if self.dry_run:
            cmd.append("--dry-run")
        return self._ssh_base() + cmd

    def resolve_and_authenticate(self, runner: Any, *, timeout: float = 3.0) -> str | None:
        """Resolve candidate addresses in order and authenticate with SSH 'true'."""
        # A cached address was authenticated when stored. The actual probe that
        # follows is the liveness check; avoid a redundant second SSH session.
        if self._cached_address:
            return self._cached_address

        def authenticate(candidate: str) -> bool:
            try:
                result = runner.run(self.auth_check_argv(candidate), timeout=timeout)
                if result.returncode == 0:
                    self._cached_address = candidate
                    return True
            except Exception:
                pass
            return False

        # 1. Prefer the configured SSH alias. It may reuse an authenticated
        # ControlMaster connection even while the hotspot is refusing new TCP
        # handshakes to the Pi's raw DHCP address.
        if self.target == DEFAULT_TARGET and authenticate(self.target):
            return self.target

        # 2. Explicit non-default target.
        if self.target != DEFAULT_TARGET and authenticate(self.target):
            return self.target

        # 3. mDNS. Authenticate immediately instead of scanning the subnet
        # first, so the common online path returns without 254 port probes.
        mdns_ip = self.resolver.mdns_resolver("ugrp1.local")
        if mdns_ip and authenticate(mdns_ip):
            return mdns_ip

        # 4. Bounded /24 SSH port scan, then authenticate only open hosts.
        local_ips = self.resolver.local_ip_getter()
        for local_ip in sorted(local_ips):
            if local_ip == "127.0.0.1" or not is_ipv4(local_ip):
                continue
            for candidate in self.resolver.scan_subnet(local_ip, local_ips):
                if candidate != mdns_ip and authenticate(candidate):
                    return candidate

        return None


class LocalTransport(ControlTransport):
    """Local execution directly on the host (e.g. on the Raspberry Pi)."""

    def __init__(
        self,
        script_path: str = DEFAULT_PI_SCRIPT,
        *,
        python_bin: str = DEFAULT_PYTHON_BIN,
        dry_run: bool = False,
    ) -> None:
        self.script_path = script_path
        self.python_bin = python_bin
        self.dry_run = dry_run

    @property
    def target_name(self) -> str:
        return "local"

    @property
    def cached_address(self) -> str | None:
        return "127.0.0.1"

    def status_argv(self) -> list[str]:
        return [self.python_bin, self.script_path, "probe"]

    def probe_argv(self) -> list[str]:
        return [self.python_bin, self.script_path, "probe"]

    def stop_argv(self) -> list[str]:
        cmd = [self.python_bin, self.script_path, "stop"]
        if self.dry_run:
            cmd.append("--dry-run")
        return cmd

    def drive_argv(self, direction: str, speed: int, duration: float) -> list[str]:
        speed = validate_chassis_speed(speed)
        cmd = [
            self.python_bin,
            self.script_path,
            "drive",
            direction,
            "--speed",
            str(speed),
            "--duration",
            format(duration, ".6g"),
        ]
        if self.dry_run:
            cmd.append("--dry-run")
        return cmd

    def servo_argv(self, servo: int, pulse: int, duration: float) -> list[str]:
        cmd = [
            self.python_bin,
            self.script_path,
            "servo",
            str(servo),
            str(pulse),
            "--duration",
            format(duration, ".6g"),
        ]
        if self.dry_run:
            cmd.append("--dry-run")
        return cmd


class MockTransport(ControlTransport):
    """In-process / safe mock execution for offline testing and development."""

    def __init__(self, *, target_name: str = "mock") -> None:
        self._target_name = target_name

    @property
    def target_name(self) -> str:
        return self._target_name

    @property
    def cached_address(self) -> str | None:
        return "127.0.0.1"

    def status_argv(self) -> list[str]:
        return [sys.executable or "python3", "-c", 'import sys, json; print(json.dumps({"ok": True, "controller": "mock", "battery_mv": 7800}))']

    def probe_argv(self) -> list[str]:
        return [sys.executable or "python3", "-c", 'import sys, json; print(json.dumps({"ok": True, "controller": "mock", "battery_mv": 7800}))']

    def stop_argv(self) -> list[str]:
        return [sys.executable or "python3", "-c", "import sys; sys.exit(0)"]

    def drive_argv(self, direction: str, speed: int, duration: float) -> list[str]:
        validate_chassis_speed(speed)
        return [sys.executable or "python3", "-c", "import sys; sys.exit(0)"]

    def servo_argv(self, servo: int, pulse: int, duration: float) -> list[str]:
        return [sys.executable or "python3", "-c", "import sys; sys.exit(0)"]


def create_transport(
    transport_type: str | None = None,
    *,
    target: str | None = None,
    script_path: str | None = None,
    python_bin: str | None = None,
    dry_run: bool | None = None,
    connect_timeout: int | None = None,
    resolver: AddressResolver | None = None,
) -> ControlTransport:
    """Factory to create a transport based on name and environment variables."""
    mode = (transport_type or os.environ.get("MASTERPI_TRANSPORT", "ssh")).strip().lower()
    resolved_target = target or os.environ.get("MASTERPI_TARGET", DEFAULT_TARGET)
    resolved_script = script_path or os.environ.get("MASTERPI_SCRIPT_PATH", DEFAULT_PI_SCRIPT)
    resolved_python = python_bin or os.environ.get("MASTERPI_PYTHON_BIN", DEFAULT_PYTHON_BIN)
    if dry_run is None:
        env_dry = os.environ.get("MASTERPI_DRY_RUN", "").strip().lower()
        resolved_dry_run = env_dry in {"1", "true", "yes"}
    else:
        resolved_dry_run = dry_run

    resolved_timeout = connect_timeout if connect_timeout is not None else DEFAULT_SSH_TIMEOUT

    if mode == "ssh":
        return SSHTransport(
            target=resolved_target,
            script_path=resolved_script,
            python_bin=resolved_python,
            connect_timeout=resolved_timeout,
            dry_run=resolved_dry_run,
            resolver=resolver,
        )
    elif mode == "local":
        return LocalTransport(
            script_path=resolved_script,
            python_bin=resolved_python,
            dry_run=resolved_dry_run,
        )
    elif mode in {"mock", "dry-run", "dry_run"}:
        return MockTransport()
    else:
        raise ValueError(f"unknown transport type: {transport_type!r} (expected 'ssh', 'local', or 'mock')")
