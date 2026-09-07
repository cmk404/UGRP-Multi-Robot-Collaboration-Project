#!/usr/bin/env bash

# Bounded, credential-free MasterPi connection diagnostic for macOS.
# It deliberately does not infer success from mDNS alone.

set -u

WAIT_SECONDS=0
INTERVAL_SECONDS=2
HOSTS="ugrp1.local masterpi1.local masterpi2.local"
LAST_RESULT="UNKNOWN"
LAST_FOUND=0

usage() {
    cat <<'EOF'
Usage: masterpi_connection_status.sh [options]

Options:
  --wait SECONDS       Retry until a target is found or the deadline expires.
  --interval SECONDS   Delay between retries (default: 2).
  --host HOST          Check one hostname or IPv4 address. May be repeated.
  --help               Show this help.

The command never prints passwords, tokens, or SSH private-key material.
EOF
}

is_integer() {
    case "${1:-}" in
        ''|*[!0-9]*) return 1 ;;
        *) return 0 ;;
    esac
}

is_ipv4() {
    printf '%s\n' "$1" | awk -F. '
        NF == 4 {
            for (i = 1; i <= 4; i++)
                if ($i !~ /^[0-9]+$/ || $i < 0 || $i > 255) exit 1
            exit 0
        }
        { exit 1 }
    '
}

resolve_host() {
    local host="$1"
    local tmp pid
    if is_ipv4 "$host"; then
        printf '%s\n' "$host"
        return 0
    fi

    # dscacheutil can block for an unbounded time on an absent .local record.
    # Use dns-sd in a short-lived child for mDNS names instead.
    case "$host" in
        *.local)
            if command -v dns-sd >/dev/null 2>&1; then
                tmp="$(mktemp -t masterpi-dns)"
                dns-sd -G v4 "$host" >"$tmp" 2>/dev/null &
                pid=$!
                sleep 1
                kill -KILL "$pid" 2>/dev/null || true
                wait "$pid" 2>/dev/null || true
                grep -Eo '([0-9]{1,3}\.){3}[0-9]{1,3}' "$tmp" || true
                rm -f "$tmp"
            fi
            ;;
        *)
            if command -v dscacheutil >/dev/null 2>&1; then
                dscacheutil -q host -a name "$host" 2>/dev/null |
                    awk '/^ip_address: / {print $2}'
            fi
            ;;
    esac

    case "$host" in
        *.local) return 0 ;;
    esac

    # Use the system resolver as a fallback for non-mDNS names when Python is
    # present. .local names returned above never reach this block.
    if command -v python3 >/dev/null 2>&1; then
        python3 - "$host" 2>/dev/null <<'PY'
import socket
import sys

socket.setdefaulttimeout(1.5)
try:
    answers = socket.getaddrinfo(sys.argv[1], None, socket.AF_INET)
except OSError:
    answers = []
for answer in answers:
    print(answer[4][0])
PY
    fi
}

port_open() {
    local host="$1"
    local port="$2"

    # macOS nc builds differ in how -w/-G behave for unroutable addresses.
    # An IP-level Python socket timeout is deterministic once the host is
    # already resolved.
    if command -v python3 >/dev/null 2>&1; then
        python3 - "$host" "$port" 2>/dev/null <<'PY'
import socket
import sys

try:
    sock = socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=1.0)
    sock.close()
except OSError:
    raise SystemExit(1)
PY
        return $?
    fi

    return 1
}

print_local_network() {
    local info ip router status airport current_network
    info="$(networksetup -getinfo Wi-Fi 2>/dev/null || true)"
    ip="$(printf '%s\n' "$info" | awk -F': ' '/^IP address:/ {print $2; exit}')"
    router="$(printf '%s\n' "$info" | awk -F': ' '/^Router:/ {print $2; exit}')"
    status="$(ifconfig en0 2>/dev/null | awk -F': ' '/status:/ {print $2; exit}')"
    airport="$(networksetup -getairportnetwork en0 2>/dev/null || true)"
    if printf '%s\n' "$airport" | grep -q 'not associated'; then
        current_network="$(system_profiler SPAirPortDataType 2>/dev/null | awk '
            /Current Network Information:/ {capture=1; next}
            capture && /^[[:space:]]+[^[:space:]].*:$/ {
                sub(/^[[:space:]]+/, "")
                sub(/:$/, "")
                print
                exit
            }
        ')"
        if [ -n "$current_network" ]; then
            airport="Current network: $current_network"
        fi
    fi

    printf 'Mac Wi-Fi: en0=%s ip=%s router=%s\n' \
        "${status:-unknown}" "${ip:-none}" "${router:-none}"
    printf 'Mac Wi-Fi service: %s\n' "${airport:-unknown}"

    if [ -z "${ip:-}" ]; then
        printf 'LOCAL_STATE=MAC_WIFI_NOT_READY\n'
    else
        printf 'LOCAL_STATE=MAC_WIFI_HAS_IPV4\n'
    fi
}

print_arp_candidates() {
    local hits
    hits="$(arp -an 2>/dev/null | grep -iE \
        '\((dc:a6:32|b8:27:eb|e4:5f:01|d8:3a:dd|28:cd:c1|2c:cf:67)' || true)"
    if [ -n "$hits" ]; then
        printf 'ARP_RASPBERRY_PI_CANDIDATES:\n%s\n' "$hits"
    else
        printf 'ARP_RASPBERRY_PI_CANDIDATES=none\n'
    fi
}

probe_targets() {
    local host ip ips ssh_state novnc_state api_state
    LAST_FOUND=0
    LAST_RESULT="PI_NOT_VISIBLE"

    for host in $HOSTS; do
        ips="$(resolve_host "$host" | awk 'NF && !seen[$0]++')"
        for ip in $ips; do
            LAST_FOUND=1
            ssh_state="closed"
            novnc_state="closed"
            api_state="closed"

            if port_open "$ip" 22; then ssh_state="open"; fi
            if port_open "$ip" 6080; then novnc_state="open"; fi
            if port_open "$ip" 8000 || port_open "$ip" 5000; then api_state="open"; fi

            printf 'TARGET=%s IP=%s SSH_22=%s NOVNC_6080=%s API_8000_OR_5000=%s\n' \
                "$host" "$ip" "$ssh_state" "$novnc_state" "$api_state"

            if [ "$ssh_state" = "open" ]; then
                LAST_RESULT="SSH_READY"
                if [ "$novnc_state" = "open" ]; then
                    LAST_RESULT="REMOTE_DESKTOP_READY"
                fi
            else
                LAST_RESULT="PI_VISIBLE_SERVICES_UNAVAILABLE"
            fi
        done
    done

    if [ "$LAST_FOUND" -eq 0 ]; then
        print_arp_candidates
    fi

    printf 'RESULT=%s\n' "$LAST_RESULT"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --wait)
            [ "$#" -ge 2 ] && is_integer "$2" || { usage >&2; exit 2; }
            WAIT_SECONDS="$2"
            shift 2
            ;;
        --interval)
            [ "$#" -ge 2 ] && is_integer "$2" || { usage >&2; exit 2; }
            INTERVAL_SECONDS="$2"
            shift 2
            ;;
        --host)
            [ "$#" -ge 2 ] || { usage >&2; exit 2; }
            if [ "$HOSTS" = "ugrp1.local masterpi1.local masterpi2.local" ]; then
                HOSTS="$2"
            else
                HOSTS="$HOSTS $2"
            fi
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
done

printf '=== MasterPi connection status ===\n'
print_local_network

start_epoch="$(date +%s)"
while :; do
    probe_targets

    if [ "$WAIT_SECONDS" -eq 0 ] || [ "$LAST_FOUND" -eq 1 ]; then
        break
    fi

    now_epoch="$(date +%s)"
    elapsed=$((now_epoch - start_epoch))
    if [ "$elapsed" -ge "$WAIT_SECONDS" ]; then
        break
    fi

    sleep "$INTERVAL_SECONDS"
done

exit 0
