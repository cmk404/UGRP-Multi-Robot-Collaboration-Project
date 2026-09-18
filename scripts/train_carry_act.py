"""Shared task-conditioned ACT; whole-episode development split, frozen vision."""
import argparse,hashlib,json,subprocess,sys,time,platform,importlib.metadata
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import torch
from harness.pair_carry_act import CarryAct,make_carry_policy,CONTEXT_KEY
from harness.pair_carry_act_contract import decode
from harness.reference_act import UPSTREAM_SHA
from harness.act_training import frozen_features
from scripts.train_act_feasibility import cached_images

def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(entries):
    rows=[];targets=[];padding=[]
    for e in entries:
        root=Path(e['root']);assert all(sha(root/k)==v for k,v in e['files'].items())
        for sequence in e['sequences'].values():
            for i,r in enumerate(sequence):
                row=dict(r)
                for view,ref in r['images'].items():
                    p=root/ref['path'];assert sha(p)==ref['sha256'];row[view+'_jpeg']=p.read_bytes()
                rows.append(row);targets.append([sequence[min(i+k,len(sequence)-1)]['action'] for k in range(8)])
                padding.append([i+k>=len(sequence) for k in range(8)])
    return rows,torch.tensor(targets,dtype=torch.float32),torch.tensor(padding,dtype=torch.bool)

@torch.no_grad()
def evaluate(policy,features,rows):
    policy.eval();pred=[]
    with frozen_features(policy):
        for i in range(0,len(rows),64):pred.extend(policy.predict_action_chunk({k:v[i:i+64] for k,v in features.items()})[:,0].tolist())
    target=np.array([r['action'] for r in rows]);values=np.array(pred);done=target[:,3]>.5;ready=values[:,3]>=.65
    groups=[3 if r['done'] else int(np.argmax(np.abs(r['action'][:3]))) for r in rows]
    mae={str(g):float(np.abs(values[np.array(groups)==g,:3]-target[np.array(groups)==g,:3]).mean()) for g in set(groups)}
    missed=float((done&~ready).sum()/max(1,done.sum()));false=float((~done&ready).sum()/max(1,(~done).sum()))
    score=missed+false+float(np.mean(list(mae.values())))
    return {'missed_done_rate':missed,'false_done_rate':false,'group_normalized_mae':mae,'selection_score':score,'samples':len(rows),'done_samples':int(done.sum())},pred

def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--steps',type=int,default=8000);p.add_argument('--seed',type=int,default=20260918);a=p.parse_args()
    assert not subprocess.check_output(['git','status','--porcelain'],cwd=ROOT)
    direct=json.loads(importlib.metadata.distribution('lerobot').read_text('direct_url.json'));assert direct['vcs_info']['commit_id']==UPSTREAM_SHA
    torch.set_num_threads(2);torch.manual_seed(a.seed);np.random.seed(a.seed);a.out.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    data=json.loads(a.dataset.read_text());rows,target,pad=load(data['train']);dev,_,_=load(data['development'])
    policy=make_carry_policy(pretrained=True)
    for v in policy.model.backbone.parameters():v.requires_grad_(False)
    cache=cached_images(policy,rows,'imagenet128');dcache=cached_images(policy,dev,'imagenet128')
    cache[CONTEXT_KEY]=torch.tensor([r['context'] for r in rows]);dcache[CONTEXT_KEY]=torch.tensor([r['context'] for r in dev])
    groups=[3 if r['done'] else int(np.argmax(np.abs(r['action'][:3]))) for r in rows];counts={g:groups.count(g) for g in set(groups)}
    weights=torch.tensor([1/counts[g] for g in groups],dtype=torch.double);generator=torch.Generator().manual_seed(a.seed)
    optimizer=torch.optim.AdamW([v for v in policy.parameters() if v.requires_grad],lr=1e-4,weight_decay=1e-4)
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,a.steps,eta_min=1e-5)
    report={'complete':False,'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'dataset_path':str(a.dataset.resolve()),'dataset_sha256':sha(a.dataset),'seed':a.seed,'steps':a.steps,'upstream_sha':UPSTREAM_SHA,'train_episodes':[e['root'] for e in data['train']],'development_episodes':[e['root'] for e in data['development']],'samples':len(rows),'groups':counts,'selection':'missed_done_rate + false_done_rate + mean direction-group normalized MAE; no test results','environment':{'python':sys.version,'platform':platform.platform(),**{k:importlib.metadata.version(k) for k in ('torch','torchvision','lerobot','numpy')}},'progress':[]}
    write(a.out/'report.json',report);best=float('inf');state=None
    for step in range(1,a.steps+1):
        idx=torch.multinomial(weights,32,replacement=True,generator=generator);batch={k:v[idx] for k,v in cache.items()};batch.update(action=target[idx],action_is_pad=pad[idx]);policy.train();optimizer.zero_grad()
        with frozen_features(policy):loss,_=policy(batch)
        assert torch.isfinite(loss);loss.backward();torch.nn.utils.clip_grad_norm_(policy.parameters(),1);optimizer.step();scheduler.step()
        if step==1 or step%500==0 or step==a.steps:
            metrics,_=evaluate(policy,dcache,dev);event={'step':step,'loss':float(loss.detach()),'development':metrics,'elapsed_s':time.monotonic()-started};report['progress'].append(event)
            if metrics['selection_score']<best:
                best=metrics['selection_score'];report['selected']=event;state={k:v.detach().cpu().clone() for k,v in policy.state_dict().items()}
            write(a.out/'report.json',report);print(json.dumps(event),flush=True)
    policy.load_state_dict(state);actor=CarryAct(policy);actor.save(a.out/'act');restored=CarryAct.load(a.out/'act')
    probes=[]
    for r in [rows[0],rows[-1],dev[0],dev[-1]]:
        expected=actor.predict(r['own_jpeg'],r['top_jpeg'],r['context']);assert restored.predict(r['own_jpeg'],r['top_jpeg'],r['context'])==expected;probes.append({'id':r['id'],'decision':expected})
    report['readback']=probes
    for split,rs,features in [('train',rows,cache),('development',dev,dcache)]:
        metrics,pred=evaluate(policy,features,rs);report[split+'_metrics']=metrics
        write(a.out/(split+'-predictions.json'),[{'id':r['id'],'target':r['action'],'prediction':v} for r,v in zip(rs,pred)])
    report['complete']=True;report['wall_s']=time.monotonic()-started;report['model_sha256']=sha(a.out/'act/model.safetensors');write(a.out/'report.json',report)
if __name__=='__main__':main()
