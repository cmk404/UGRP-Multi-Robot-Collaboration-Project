"""Owned ACT worker; wire hashes can be reconstructed from saved actor inputs."""
import base64,hashlib,json,selectors,subprocess
from pathlib import Path
from harness.recovery_act_client import RecoveryClient,interpreter_path
class CarryClient(RecoveryClient):
    def __init__(self,python,model_dir,timeout_s=45):
        self.timeout_s=timeout_s;root=Path(__file__).resolve().parents[1]
        self.process=subprocess.Popen([str(interpreter_path(python)),str(root/'scripts/pair_carry_act_worker.py'),'--model-dir',str(model_dir)],cwd=root,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        self.selector=selectors.DefaultSelector();self.selector.register(self.process.stdout,selectors.EVENT_READ)
        try:
            if self._read()!={'ready':True,'runtime_inputs':['own_rgb','top_rgb','context']}:raise ValueError('worker startup')
        except BaseException:self.close();raise
    def predict(self,own,top,context):
        request={'own_rgb':base64.b64encode(own).decode('ascii'),'top_rgb':base64.b64encode(top).decode('ascii'),'context':context}
        payload=json.dumps(request)+'\n';self.last_request_sha256=hashlib.sha256(payload.encode()).hexdigest()
        self.process.stdin.write(payload);self.process.stdin.flush();return self._read()
