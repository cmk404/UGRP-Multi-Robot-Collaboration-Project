"""Structured component failure provenance, without changing control actions."""


class ComponentExecutionError(RuntimeError):
    def __init__(self, component, cause):
        self.component=component
        super().__init__(f'{component}: {type(cause).__name__}: {cause}')
