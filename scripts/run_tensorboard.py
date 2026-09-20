#!/usr/bin/env python3
"""Run official TensorBoard plus a tiny read-only original-MP4 server on loopback.

Use scripts/ugrp_session.py to own/stop the complete process group.
"""
import argparse
from pathlib import Path
import signal
import subprocess
import sys
import threading

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.tensorboard_tools.media import make_server


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--logdir',type=Path,required=True)
    parser.add_argument('--port',type=int,default=6006)
    parser.add_argument('--media-port',type=int,default=6007)
    args=parser.parse_args()
    if not args.logdir.is_dir(): parser.error('logdir does not exist; export records first')
    if args.port==args.media_port: parser.error('TensorBoard and media ports must differ')
    server=make_server(args.logdir.resolve(),args.media_port)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    child=None
    def stop(_signum,_frame):
        raise KeyboardInterrupt()
    previous=signal.signal(signal.SIGTERM,stop)
    try:
        child=subprocess.Popen([sys.executable,'-m','tensorboard.main','--logdir',str(args.logdir.resolve()),
            '--host','127.0.0.1','--port',str(args.port),'--reload_interval','5',
            '--samples_per_plugin','scalars=0,images=100,tensors=200'])
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
