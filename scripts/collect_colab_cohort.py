"""Finite incremental collection of finalized cohort checkpoints from Colab."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.cloud_collection import RemoteHealth,write_json,verify_checkpoint


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True);p.add_argument('--remote',required=True)
    p.add_argument('--source-sha',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=14400)
    a=p.parse_args()
    if not 0<a.seconds<=14400: p.error('invalid finite deadline')
    import requests
    from colab_cli.contents import ContentsClient
    from colab_cli.state import StateStore
    original=requests.request
    def bounded(*args,**kwargs):
        kwargs.setdefault('timeout',(10,45));return original(*args,**kwargs)
    requests.request=bounded
    a.output.mkdir(parents=True,exist_ok=False)
    client=ContentsClient(StateStore().get(a.session))
    health=RemoteHealth();state={'started_unix':time.time(),'complete':False,'trials':{},'source_sha':a.source_sha}
    deadline=time.monotonic()+a.seconds
    state.update(health.state)
    write_json(a.output/'collection-status.json',state)
    while time.monotonic()<deadline:
        try:
            client.download(a.remote+'/state-model-v4.json',str(a.output/'remote-state.json'))
            remote=json.loads((a.output/'remote-state.json').read_text())
            if remote['source_sha']!=a.source_sha:raise ValueError('remote source mismatch')
            health.success('terminal' if remote['complete'] else 'running')
            state.update(health.state,remote=remote)
            write_json(a.output/'collection-status.json',state)
            phases=set(remote.get('exports',{})) | ({remote['phase']} if remote.get('phase') else set())
            for phase in sorted(phases):
                if phase not in ('development','holdout','regression','ablation','continuous'):raise ValueError('unknown phase')
                root=a.remote+'/source/outputs/gpu-'+phase+'/result'
                try:entries=client.list_dir(root+'/checkpoints')['content']
                except FileNotFoundError:continue
                local=a.output/phase;local.mkdir(exist_ok=True)
                protocol=local/'protocol.json'
                if not protocol.exists():client.download(root+'/protocol.json',str(protocol))
                for entry in entries:
                    name=entry['name']
                    if not name.endswith('.json') or Path(name).name!=name:continue
                    identity=phase+'/'+name[:-5]
                    if identity in state['trials']:continue
                    path=local/name;client.download(root+'/checkpoints/'+name,str(path))
                    manifest=json.loads(path.read_text());archive=manifest['archive']
                    if Path(archive).name!=archive:raise ValueError('unsafe checkpoint path')
                    temp=local/(archive+'.part');client.download(root+'/checkpoints/'+archive,str(temp))
                    result=verify_checkpoint(temp,manifest,a.source_sha);temp.replace(local/archive)
                    state['trials'][identity]={'archive':str(local/archive),'sha256':manifest['sha256'],
                        'success':result['success'],'policy':result['policy'],'wall_s':result['wall_s']}
                    write_json(a.output/'collection-status.json',state)
            health.success('terminal' if remote['complete'] else 'running')
            state.update(health.state,remote=remote)
            if remote['complete']:
                client.download('/content/model_v4_supervisor.log',str(a.output/'remote.log'))
                planned=sum(len(json.loads(p.read_text())['jobs']) for p in a.output.glob('*/protocol.json'))
                state.update(complete=True,collection_complete=True,ended_unix=time.time(),planned_trials=planned,
                    cohort_complete=not remote.get('stop_reason') and not remote.get('error_type')
                    and len(phases)==5 and len(state['trials'])==planned)
            write_json(a.output/'collection-status.json',state)
            if state['complete']:return 0
        except Exception as exc:
            exhausted=health.failure(exc);state.update(health.state)
            if exhausted:state.update(stop_reason='remote_access_or_collection_failed',ended_unix=time.time())
            write_json(a.output/'collection-status.json',state)
            if exhausted:return 1
        time.sleep(15)
    state.update(stop_reason='collection_deadline',ended_unix=time.time())
    write_json(a.output/'collection-status.json',state)
    return 1

if __name__=='__main__':raise SystemExit(main())
