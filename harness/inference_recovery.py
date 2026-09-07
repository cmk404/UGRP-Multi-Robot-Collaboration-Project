"""Retry scheduling only; the caller must capture fresh images for every retry."""


def transport_error(exc):
    """The planner wraps completer errors; retain the typed original cause."""
    seen = set()
    while exc is not None and id(exc) not in seen:
        seen.add(id(exc))
        if hasattr(exc, 'error_kind') and hasattr(exc, 'retryable'):
            return exc
        exc = exc.__cause__
    return None


class InferenceRecovery:
    def __init__(self, max_transient_failures=5):
        self.max_transient_failures = max_transient_failures
        self.consecutive = 0
        self.total_errors = 0
        self.next_wall = 0.0

    def success(self):
        self.consecutive = 0
        self.next_wall = 0.0

    def failure(self, exc, now_wall):
        self.total_errors += 1
        self.consecutive += 1
        retryable = bool(getattr(exc, 'retryable', False))
        # Rate limits need a substantially longer cooldown than a dropped
        # connection. Honor an explicit server deadline when provided.
        rate_limited = getattr(exc, 'http_status', None) == 429
        delay = (min(120.0, 30.0 * 2.0 ** (self.consecutive-1)) if rate_limited
                 else min(8.0, 2.0 ** (self.consecutive-1))) if retryable else 0.0
        server_delay = getattr(exc, 'retry_after_s', None)
        if retryable and isinstance(server_delay, (int, float)) and 0 <= server_delay <= 86400:
            delay = max(delay, float(server_delay))
        retry = retryable and self.consecutive < self.max_transient_failures
        self.next_wall = float(now_wall)+delay if retry else 0.0
        return {'error_kind': getattr(exc, 'error_kind', type(exc).__name__),
                'retryable': retryable, 'retry_scheduled': retry,
                'retry_delay_wall_s': delay if retry else 0.0,
                'consecutive_errors': self.consecutive,
                'http_status': getattr(exc, 'http_status', None),
                'requires_fresh_observation': retry}

    def ready(self, now_wall):
        return float(now_wall) >= self.next_wall
