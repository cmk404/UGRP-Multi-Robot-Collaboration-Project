"""Dependency-free contract for bounded render snapshot admission."""


class SnapshotBackpressure(RuntimeError):
    """All bounded render snapshots are in flight; retry on a later cadence."""
