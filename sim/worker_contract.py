"""Versioned compatibility contract for remote production SIM workers."""

REMOTE_WORKER_CONTRACT = "masterpi-v2-coela-traffic-budget-v9"


def worker_contract_compatible(value: object) -> bool:
    return str(value or "").strip() == REMOTE_WORKER_CONTRACT
