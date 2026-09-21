"""Collection health cannot grant permission to terminate an experiment."""


def cleanup_ready(*, deadline_reached, remote_complete, collection_complete):
    return bool(deadline_reached or (remote_complete and collection_complete))
