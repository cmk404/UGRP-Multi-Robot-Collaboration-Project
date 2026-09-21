"""Minimal read-only original-video companion for TensorBoard Text links."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import html
import json
from pathlib import Path
import re
from urllib.parse import urlsplit


def media_registry(logdir):
    registry = {}
    for path in Path(logdir).rglob('manifest.json'):
        if path.is_symlink(): continue
        try: manifest = json.loads(path.read_text())
        except (OSError, ValueError): continue
        if manifest.get('schema') != 'ugrp.tensorboard-export.v1' or manifest.get('complete') is not True: continue
        source = Path(manifest['source']).resolve()
        for entry in manifest.get('videos', []):
            p = Path(entry.get('path', ''))
            if (p.name not in ('motion.mp4', 'execution.mp4') or p.is_symlink()
                    or not p.resolve().is_relative_to(source)): continue
            if re.fullmatch(r'[0-9a-f]{20}', entry.get('id', '')):
                registry[entry['id']] = entry
    return registry


def handler_for(registry):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass

        def headers_for(self, status, mime, size):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(size))
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', "default-src 'none'; media-src 'self'; style-src 'unsafe-inline'; frame-ancestors 'none'; base-uri 'none'")

        def message(self, status, message):
            data = message.encode()
            self.headers_for(status, 'text/plain; charset=utf-8', len(data));self.end_headers();self.wfile.write(data)

        def do_GET(self):
            allowed = {f'localhost:{self.server.server_port}', f'127.0.0.1:{self.server.server_port}'}
            if self.headers.get('Host') not in allowed:
                self.message(403, 'Local host only');return
            match = re.fullmatch(r'/(video|raw)/([0-9a-f]{20})', urlsplit(self.path).path)
            if not match or match[2] not in registry:
                self.message(404, 'No registered video');return
            entry = registry[match[2]];path = Path(entry['path'])
            try:
                if path.is_symlink() or path.resolve() != path: self.message(404, 'Video moved');return
                st = path.stat()
                if st.st_size != entry['size'] or st.st_mtime_ns != entry['mtime_ns']:
                    self.message(409, 'Video changed since export; export a new snapshot');return
                if match[1] == 'video':
                    data = ('<!doctype html><html lang="ko"><meta charset="utf-8"><title>UGRP 원본 영상</title>'
                        '<style>body{margin:24px;font:14px -apple-system,sans-serif;color:#333}video{display:block;width:min(1100px,100%);max-height:80vh;background:#20252b}p{color:#777;font-size:12px;overflow-wrap:anywhere}</style>'
                        f'<h3>{html.escape(path.parent.name)} / {html.escape(path.name)}</h3>'
                        f'<video controls src="/raw/{match[2]}"></video>'
                        '<p>원본 MP4 · 플레이어 시각과 SIM 시각은 자동 동기화하지 않습니다.</p>'
                        f'<p>{html.escape(str(path))}</p></html>').encode()
                    self.headers_for(200, 'text/html; charset=utf-8', len(data));self.end_headers();self.wfile.write(data);return
                size=st.st_size;start,end,status=0,size-1,200
                requested=self.headers.get('Range')
                if requested:
                    m=re.fullmatch(r'bytes=(\d*)-(\d*)',requested)
                    if not m or not any(m.groups()): self.message(416, 'Invalid range');return
                    a,b=m.groups()
                    if a: start,end=int(a),min(int(b),size-1) if b else size-1
                    else: start,end=max(0,size-int(b)),size-1
                    if start>end or start>=size:
                        self.headers_for(416,'text/plain',0);self.send_header('Content-Range',f'bytes */{size}');self.end_headers();return
                    status=206
                self.headers_for(status,'video/mp4',max(0,end-start+1));self.send_header('Accept-Ranges','bytes')
                if status==206: self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
                self.end_headers()
                with path.open('rb') as f:
                    f.seek(start);remaining=end-start+1
                    while remaining>0:
                        block=f.read(min(remaining,256*1024))
                        if not block: break
                        self.wfile.write(block);remaining-=len(block)
            except (BrokenPipeError,ConnectionResetError): pass
            except OSError: self.message(404,'Original video unavailable')
    return Handler


def make_server(logdir, port):
    return ThreadingHTTPServer(('127.0.0.1', port),handler_for(media_registry(logdir)))
