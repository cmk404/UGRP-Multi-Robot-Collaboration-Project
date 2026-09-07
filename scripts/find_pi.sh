#!/usr/bin/env bash

# Backward-compatible entry point. The old watcher waited forever and printed
# a credential; the bounded diagnostic is now the single implementation.

set -u
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WAIT_SECONDS="${1:-120}"

exec "$SCRIPT_DIR/masterpi_connection_status.sh" --wait "$WAIT_SECONDS"
