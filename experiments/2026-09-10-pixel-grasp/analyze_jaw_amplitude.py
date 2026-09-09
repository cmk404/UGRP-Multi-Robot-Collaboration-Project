"""Output-only RGB replay and calibration comparison, no control feedback."""
from pathlib import Path
import sys,json,hashlib
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from harness.camera_gripper_motion import GripperMotionTracker

def analyze(root):
 p=Path(root);r=json.loads((p/'result.json').read_text())
 prior=2000
 for a in r['replay_actions']:
  if a.get('servo_id')==1:prior=a['pulse']
 initial=next(c for c in r['captures'] if c['phase']=='post-replay')
 t=GripperMotionTracker(initial_gripper_pulse=prior);t.update((p/initial['files']['top']['path']).read_bytes(),None)
 measurements=[]
 for command,c in zip(r['command_sequence'],[c for c in r['captures'] if c['phase']=='after']):
  raw=(p/c['files']['top']['path']).read_bytes();assert hashlib.sha256(raw).hexdigest()==c['files']['top']['sha256']
  measurements.append({'command':command,'measurement':t.update(raw,command)})
 return {'root':str(p),'source_sha':r['source_sha'],'close_pulse':r['config']['close_pulse'],'post_replay':initial['files'],'measurements':measurements,'error':r['error'],'wall_elapsed_s':r['wall_elapsed_s']}
if __name__=='__main__':print(json.dumps([analyze(p) for p in sys.argv[1:]],indent=2))
