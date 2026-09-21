"""One finite Colab continuation; collection failures never stop its worker.

Creates exactly one explicitly named runtime. No automatic VM reallocation,
public endpoints, recurring monitor, or model credentials on the runtime.
"""
import argparse
import base64
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.cloud_collection import digest,write_json
from scripts.colab_live_contents import LiveContentsClient
from scripts.colab_job_lifecycle import AssignmentLoss,cleanup_ready
from scripts.colab_simulation_cli import setup_code


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--actor-bundle',type=Path,required=True)
    p.add_argument('--resume-manifest',type=Path,required=True)
    p.add_argument('--proxy-script',type=Path,required=True)
    p.add_argument('--relay-script',type=Path,required=True)
    p.add_argument('--keychain-helper',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=14400)
    p.add_argument('--benchmark-relay',action='store_true')
    a=p.parse_args()
    if not 1200 <= a.seconds <= 14400:p.error('finite budget must be 1200..14400 seconds')
    from colab_cli.state import StateStore
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    base='/content/'+a.session;py=base+'/sim-env/bin/python'
    record=json.loads((a.actor_bundle/'source-manifest.json').read_text())
    if digest(a.actor_bundle/'source.tar.gz')!=record['sha256']:raise ValueError('actor archive changed')
    manifest=json.loads(a.resume_manifest.read_text())
    if manifest['actor_source_sha']!=record['source_sha']:raise ValueError('resume actor mismatch')
    state={'started_unix':time.time(),'stage':'create','complete':False,'session':a.session,
           'source_sha':record['source_sha'],'control_source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()}
    deadline=time.monotonic()+a.seconds;children=[];allowed_cleanup=False;session=None
    assignment_loss=AssignmentLoss();lost=False
    def save():write_json(out/'controller-status.json',state)
    def command(args,name,timeout=180):
        with (out/(name+'.log')).open('w') as log:
            result=subprocess.run(list(map(str,args)),stdout=log,stderr=subprocess.STDOUT,timeout=timeout)
        if result.returncode:raise RuntimeError(name+' failed')
    def spawn(args,name,**kwargs):
        with (out/(name+'.log')).open('w') as log:
            child=subprocess.Popen(list(map(str,args)),stdout=log,stderr=subprocess.STDOUT,**kwargs)
        children.append(child);return child
    save()
    try:
        if StateStore().get(a.session) is not None:raise ValueError('session name already exists')
        command(['colab','new','-s',a.session,'--gpu','T4'],'create')
        session=StateStore().get(a.session);client=LiveContentsClient(session)
        if a.benchmark_relay:
            from scripts.benchmark_colab_relay import benchmark
            state['stage']='relay_benchmark';save()
            try:benchmark(session,out/'relay-benchmark.json')
            except Exception as exc:
                write_json(out/'relay-benchmark.json',{'verified':False,'error_type':type(exc).__name__})
        parts=[]
        with (a.actor_bundle/'source.tar.gz').open('rb') as stream:
            while block:=stream.read(4*1024*1024):
                name=base+'.part-'+str(len(parts));parts.append(name)
                client._request('PUT',name,json_data={'name':Path(name).name,'path':name,'type':'file','format':'base64','content':base64.b64encode(block).decode(),'chunk':1})
        setup=out/'setup.py';setup.write_text(setup_code(base,record['sha256']))
        client.upload(str(setup),'/content/recovery-setup.py')
        client.upload(str(a.resume_manifest),'/content/recovery-manifest.json')
        runner=ROOT/'scripts/run_frozen_skill_resume.py'
        client.upload(str(runner),'/content/recovery-runner.py')
        bootstrap=out/'bootstrap.py'
        bootstrap.write_text('import subprocess,sys,hashlib; from pathlib import Path\n'+
            'assert hashlib.sha256(Path("/content/recovery-runner.py").read_bytes()).hexdigest()=='+repr(digest(runner))+'\n'+
            'with open('+repr(base+'.tar.gz')+',"wb") as dest:\n for part in '+repr(parts)+':dest.write(Path(part).read_bytes())\n'+
            'subprocess.run([sys.executable,"/content/recovery-setup.py"],check=True)\n'+
            'subprocess.run('+repr([py,base+'/source/scripts/setup_colab_egl.py','--python',py,'--output',base+'/egl.json'])+',check=True)\n'+
            'with open("/content/model_v4_supervisor.log","w") as log:\n subprocess.run('+repr(['timeout','--signal=INT','--kill-after=30',str(a.seconds-300),py,'/content/recovery-runner.py','--base',base,'--manifest','/content/recovery-manifest.json','--manifest-sha',digest(a.resume_manifest),'--seconds',str(a.seconds-900)])+',stdout=log,stderr=subprocess.STDOUT,check=True)\n')
        client.upload(str(bootstrap),'/content/recovery-bootstrap.py')
        foreground=out/'foreground.py';foreground.write_text('import subprocess;subprocess.run(["python","/content/recovery-bootstrap.py"],check=True)\n')
        execution=spawn(['colab','exec','-s',a.session,'-f',foreground,'--timeout',str(a.seconds)],'execution')
        spawn([sys.executable,a.proxy_script],'proxy')
        spawn([sys.executable,a.relay_script,'--session',a.session,'--remote',base+'/mailbox','--keychain-helper',a.keychain_helper,'--output',out/'relay','--seconds',str(a.seconds),'--max-calls','16000'],'relay')
        collector=spawn([sys.executable,ROOT/'scripts/collect_colab_cohort.py','--session',a.session,'--remote',base,'--source-sha',record['source_sha'],'--output',out/'collected','--seconds',str(a.seconds)],'collector')
        state['stage']='running';save()
        while True:
            collection={}
            try:collection=json.loads((out/'collected/collection-status.json').read_text())
            except (FileNotFoundError,json.JSONDecodeError):pass
            remote=collection.get('remote',{})
            state.update(execution_exit_code=execution.poll(),collector_exit_code=collector.poll(),
                         remote_complete=remote.get('complete',False),phase=remote.get('phase'),
                         collection_complete=collection.get('collection_complete',False),
                         collection_health=collection.get('remote_state','starting'),
                         recovered_trials=len(collection.get('trials',{})),last_observed_unix=time.time())
            # A failed download alone never authorizes stopping a runtime.
            # Confirm assignment loss independently, without allocating a replacement.
            if state['collection_health']=='unreachable':
                from colab_cli.common import state as colab_state
                try:present=any(item.endpoint==session.endpoint for item in colab_state.client.list_assignments())
                except Exception:present=None
                lost=assignment_loss.observe(present,time.monotonic())
            else:assignment_loss.observe(True,time.monotonic())
            state['confirmed_assignment_lost']=lost
            allowed_cleanup=cleanup_ready(deadline_reached=time.monotonic()>=deadline,
                remote_complete=state['remote_complete'],collection_complete=state['collection_complete'])
            save()
            if allowed_cleanup or lost:break
            time.sleep(10)
        state.update(complete=bool(state['remote_complete'] and state['collection_complete']),
                     stage='assignment_lost' if lost else ('collected' if state['collection_complete'] else 'deadline'))
    except Exception as exc:
        state.update(stage='controller_error',error_type=type(exc).__name__)
        raise
    finally:
        # Never use a collector exception/exit to authorize runtime termination.
        if allowed_cleanup or lost:
            for child in children:
                if child.poll() is None:child.terminate()
            for child in children:
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:child.kill();child.wait()
            current=StateStore().get(a.session)
            if not lost and session and current and current.endpoint==session.endpoint:
                try:command(['colab','stop','-s',a.session],'stop',60)
                except Exception as exc:state['cleanup_error_type']=type(exc).__name__
        state.update(ended_unix=time.time(),cleanup_authorized=allowed_cleanup);save()
    return 0 if state['complete'] else 1


if __name__=='__main__':raise SystemExit(main())
