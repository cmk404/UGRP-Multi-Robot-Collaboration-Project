#!/usr/bin/env python3
"""Run official TensorBoard plus a tiny read-only original-MP4 server on loopback.

Use scripts/ugrp_session.py to own/stop the complete process group.
"""
import argparse
import errno
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.tensorboard_tools.media import make_server


def resolve_logdir(logdir, prefer_latest_collection=False):
    """Resolve a review logdir without treating export time as experiment time."""
    root=Path(logdir).resolve()
    if not prefer_latest_collection: return root
    candidates=[]
    try: children=list(root.iterdir())
    except OSError: return root
    for child in children:
        if not child.is_dir() or child.is_symlink(): continue
        collection=child/'collection.json'
        if not collection.is_file(): continue
        try:
            data=json.loads(collection.read_text())
            if not isinstance(data.get('exported'),list) or not data['exported']: continue
            candidates.append((collection.stat().st_mtime_ns,child.name,child.resolve()))
        except (OSError,ValueError,TypeError): continue
    return max(candidates)[2] if candidates else root


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--logdir',type=Path,required=True)
    parser.add_argument('--prefer-latest-collection',action='store_true',
        help='when logdir contains exported review collections, open the newest export snapshot instead of every collection')
    parser.add_argument('--port',type=int,default=6006)
    parser.add_argument('--media-port',type=int,default=6007)
    args=parser.parse_args()
    logdir=resolve_logdir(args.logdir,args.prefer_latest_collection)
    if not logdir.is_dir(): parser.error('logdir does not exist; export records first')
    if args.port==args.media_port: parser.error('TensorBoard and media ports must differ')
    try: server=make_server(logdir,args.media_port)
    except OSError as e:
        if e.errno==errno.EADDRINUSE:
            parser.error(f'media port {args.media_port} is already in use; stop the existing review server or use a matching export/media port')
        raise
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    child=None
    def stop(_signum,_frame):
        raise KeyboardInterrupt()
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        print(f'UGRP TensorBoard logdir: {logdir}',flush=True)
        child=subprocess.Popen([sys.executable,'-m','tensorboard.main','--logdir',str(logdir),
            '--host','127.0.0.1','--port',str(args.port),'--reload_interval','5',
            '--samples_per_plugin','scalars=10000,images=100,tensors=200'])
        print(f'TensorBoard http://127.0.0.1:{args.port} · Original video links: 127.0.0.1:{args.media_port}',flush=True)
        return child.wait()
    except KeyboardInterrupt: return 0
    finally:
        signal.signal(signal.SIGTERM,previous)
        if child and child.poll() is None:
            child.terminate()
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: child.kill();child.wait()
        server.shutdown();server.server_close();thread.join(timeout=3)


if __name__=='__main__': raise SystemExit(main())
