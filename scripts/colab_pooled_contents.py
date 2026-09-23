"""Persistent HTTPS connections to the existing authorized Colab Contents API."""
from urllib.parse import quote


def pooled_factory(session):
    import requests
    from colab_cli.contents import ContentsClient, get_status_code

    class PooledContents(ContentsClient):
        def __init__(self, state):
            super().__init__(state)
            self.http = requests.Session()

        def _request(self, method, path, params=None, json_data=None):
            url = self.base_url+'/api/contents/'+quote(path.strip('/'), safe='/')
            query = {'authuser': '0', 'colab-runtime-proxy-token': self.token}
            if params:
                query.update(params)
            response = self.http.request(method, url, params=query, json=json_data, timeout=(5, 10))
            if get_status_code(response) == 404:
                raise FileNotFoundError(path)
            response.raise_for_status()
            return None if method == 'DELETE' else response.json()

        def close(self):
            self.http.close()

    return PooledContents(session)
