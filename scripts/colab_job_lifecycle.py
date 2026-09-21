"""Collection health cannot grant permission to terminate an experiment."""


def cleanup_ready(*, deadline_reached, remote_complete, collection_complete):
    return bool(deadline_reached or (remote_complete and collection_complete))


class AssignmentLoss:
    """Only repeated successful server listings can confirm a lost runtime."""
    def __init__(self):
        self.first_missing = None
        self.missing_checks = 0

    def observe(self, present, now):
        if present is not False:
            self.first_missing = None
            self.missing_checks = 0
            return False
        if self.first_missing is None:self.first_missing = now
        self.missing_checks += 1
        return self.missing_checks >= 3 and now-self.first_missing >= 60
