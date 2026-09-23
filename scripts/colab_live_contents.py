"""Refresh credentials for the SAME assigned runtime; never allocate or restart it."""
import time


class RuntimeAssignmentMissing(RuntimeError):
    pass


class LiveContentsClient:
    def __init__(self, session, *, assignments=None, factory=None, clock=time.monotonic):
        if assignments is None:
            from colab_cli.common import state
            assignments = state.client.list_assignments
        if factory is None:
            from colab_cli.contents import ContentsClient
            factory = ContentsClient
        self.session, self.assignments, self.factory = session, assignments, factory
        self.clock = clock
        self.client = factory(session)
        self.refresh_at = 0
        self.refreshed_at = float('-inf')
        self.last_operation = None
        self.refresh_count = 0

    def refresh(self):
        matches = [a for a in self.assignments() if a.endpoint == self.session.endpoint]
        if len(matches) != 1:
            raise RuntimeAssignmentMissing('the owned runtime is not assigned')
        info = matches[0].runtime_proxy_info
        updated = self.session.model_copy(update={'token': info.token, 'url': info.url})
        old = self.client
        self.client = self.factory(updated)
        close = getattr(old, 'close', None)
        if callable(close):
            close()
        self.refreshed_at = self.clock()
        self.refresh_at = self.refreshed_at + max(1, min(300, info.token_expires_in_seconds / 2))
        self.refresh_count += 1

    def _call(self, method, *args, **kwargs):
        # Store only the relative runtime path, never a token-bearing URL/error.
        self.last_operation = {'method': method, 'path': str(args[1] if method == '_request' else args[0])}
        if self.clock() >= self.refresh_at:
            self.refresh()
        try:
            return getattr(self.client, method)(*args, **kwargs)
        except Exception as exc:
            code = getattr(getattr(exc, 'response', None), 'status_code', None)
            if (isinstance(exc, FileNotFoundError) or code in (401, 403, 404)) and self.clock() - self.refreshed_at >= 5:
                self.refresh()
                return getattr(self.client, method)(*args, **kwargs)
            raise

    def _request(self, *args, **kwargs):
        return self._call('_request', *args, **kwargs)

    def download(self, *args, **kwargs):
        return self._call('download', *args, **kwargs)

    def upload(self, local, remote):
        return self._request('PUT', remote, json_data=self._upload_payload(local, remote))

    @staticmethod
    def _upload_payload(local, remote):
        import base64
        from pathlib import Path
        return {'name': Path(remote).name, 'path': remote, 'type': 'file', 'format': 'base64',
                'content': base64.b64encode(Path(local).read_bytes()).decode(), 'chunk': 1}

    def list_dir(self, path):
        return self._request('GET', path)

    def close(self):
        close = getattr(self.client, 'close', None)
        if callable(close):
            close()
