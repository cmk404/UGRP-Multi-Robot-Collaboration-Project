#!/usr/bin/env python3
"""Train one recovery ACT condition; cache only frozen image features."""
import argparse,json,hashlib,subprocess,sys,time,platform,importlib.metadata
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
import torch
from harness.recovery_act import RecoveryAct,make_recovery_policy,decode,SCALES
from harness.reference_act import IMAGE_KEYS,UPSTREAM_SHA
from harness.act_training import frozen_features
from scripts.train_act_feasibility import cached_images

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')

def load(spec):
    trajectories={r:{} for r in ('r1','r3')};provenance=[]
    for entry in spec:
        root=Path(entry['root'])
        for cid in entry['cases']:
            folder=root/cid;report=json.loads((folder/'result.json').read_text())
            if not report['training_eligible']:raise ValueError('ineligible training trajectory '+cid)
            actors=json.loads((folder/'actor_samples.json').read_text());labels={r['id']:r for r in json.loads((folder/'teacher_labels.json').read_text())}
            if len(labels)!=len(actors) or any(l['heldout_region'] for l in labels.values()):raise ValueError('invalid or contaminated labels')
            key=str(folder.resolve())
            provenance.append({'path':key,'result_sha256':sha(folder/'result.json'),'actor_sha256':sha(folder/'actor_samples.json'),'label_sha256':sha(folder/'teacher_labels.json')})
            for a in actors:
                label=labels[a['id']]
                if a['case_id']!=label['case_id'] or a['robot_id']!=label['robot_id'] or set(a['images'])!={'own','top'}:raise ValueError('input/label mismatch')
                row={'sample_id':a['id'],'case_key':key,'stop':label['target']['stop']}
                for view in ('own','top'):
                    im=a['images'][view];p=(folder/im['path']).resolve()
                    if not p.is_relative_to(folder.resolve()) or sha(p)!=im['sha256']:raise ValueError('image hash mismatch')
                    row[view+'_jpeg']=p.read_bytes()
                row['target']=[label['target'][k]/scale for k,scale in zip(('forward','left','turn'),SCALES)]+[float(row['stop'])]
                trajectories[a['robot_id']].setdefault(key,[]).append(row)
    return trajectories,provenance

def chunks(trajectories):
    rows=[];targets=[];padding=[]
    for sequence in trajectories.values():
        for i,row in enumerate(sequence):
            rows.append(row);targets.append([sequence[min(i+k,len(sequence)-1)]['target'] for k in range(4)])
            padding.append([i+k>=len(sequence) for k in range(4)])
    return rows,torch.tensor(targets,dtype=torch.float32),torch.tensor(padding,dtype=torch.bool)

@torch.no_grad()
def evaluate(policy,cache,rows):
    policy.eval();values=[]
    with frozen_features(policy):
        for i in range(0,len(rows),64):values.extend(policy.predict_action_chunk({k:v[i:i+64] for k,v in cache.items()})[:,0].tolist())
    expected=np.array([r['target'] for r in rows]);pred=np.array(values)
    ready=np.array([decode(v)['ready'] for v in values]);stops=expected[:,3]>.5
    missed=int((stops&~ready).sum());false=int((~stops&ready).sum());mae=np.abs(pred[:,:3]-expected[:,:3]).mean(axis=0)
    metrics={'samples':len(rows),'stop_samples':int(stops.sum()),'missed_ready':missed,'false_ready':false,'normalized_action_mae':mae.tolist(),
             'selection_score':missed/max(1,stops.sum())+10*false/max(1,(~stops).sum())+float(mae.mean())}
    return metrics,values

def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--steps',type=int,default=10000);a=p.parse_args()
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT):raise ValueError('commit before training')
    direct=json.loads(importlib.metadata.distribution('lerobot').read_text('direct_url.json'))
    if direct.get('vcs_info',{}).get('commit_id')!=UPSTREAM_SHA:raise ValueError('wrong upstream')
    torch.set_num_threads(2);a.out.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    spec=json.loads(a.dataset.read_text());train,provenance=load(spec['train']);dev,devprovenance=load(spec['development'])
    if set(train['r1'])&set(dev['r1']):raise ValueError('train/dev overlap')
    report={'complete':False,'source_sha':source,'dataset_sha256':sha(a.dataset),'dataset':spec,'sources':provenance,'development_sources':devprovenance,'seed':a.seed,'steps':a.steps,'batch':32,'upstream_sha':UPSTREAM_SHA,'robots':{},'environment':{'python':sys.version,'platform':platform.platform(),**{k:importlib.metadata.version(k) for k in ('torch','torchvision','lerobot','numpy')}},'external_model_calls':0}
    write(a.out/'report.json',report)
    for rid in ('r1','r3'):
        torch.manual_seed(a.seed);np.random.seed(a.seed);folder=a.out/rid;folder.mkdir()
        rows,targets,padding=chunks(train[rid]);drows,_,_=chunks(dev[rid]);policy=make_recovery_policy(pretrained=True)
        for v in policy.model.backbone.parameters():v.requires_grad_(False)
        features=cached_images(policy,rows,'imagenet128');dfeatures=cached_images(policy,drows,'imagenet128')
        groups=[3 if r['stop'] else (4 if r['target'][0]<0 and abs(r['target'][0])>=max(abs(v) for v in r['target'][1:3]) else int(np.argmax(np.abs(r['target'][:3])))) for r in rows]
        counts={g:groups.count(g) for g in set(groups)};weights=torch.tensor([1/counts[g] for g in groups],dtype=torch.double)
        generator=torch.Generator().manual_seed(a.seed)
        optimizer=torch.optim.AdamW([v for v in policy.parameters() if v.requires_grad],lr=1e-4,weight_decay=1e-4)
        schedule=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,a.steps,eta_min=1e-5)
        best=float('inf');state=None;selected=None;progress=[];begin=time.monotonic()
        for step in range(1,a.steps+1):
            idx=torch.multinomial(weights,32,replacement=True,generator=generator)
            batch={k:v[idx] for k,v in features.items()};batch.update(action=targets[idx],action_is_pad=padding[idx])
            policy.train();optimizer.zero_grad()
            with frozen_features(policy):loss,_=policy(batch)
            if not torch.isfinite(loss):raise ValueError('nonfinite loss')
            loss.backward();torch.nn.utils.clip_grad_norm_(policy.parameters(),1.);optimizer.step();schedule.step()
            if step==1 or step%500==0 or step==a.steps:
                metrics,_=evaluate(policy,dfeatures,drows);event={'step':step,'loss':float(loss.detach()),'development':metrics,'elapsed_s':time.monotonic()-begin};progress.append(event);write(folder/'progress.json',progress)
                print(json.dumps({'robot':rid,**event}),flush=True)
                if metrics['selection_score']<best:best=metrics['selection_score'];selected=event;state={k:v.detach().cpu().clone() for k,v in policy.state_dict().items()}
        policy.load_state_dict(state);actor=RecoveryAct(policy);actor.save(folder/'act');restored=RecoveryAct.load(folder/'act')
        probes=[]
        for group in (rows,drows):
            for stop in (False,True):
                row=next(r for r in group if r['stop']==stop);expected=actor.predict(row['own_jpeg'],row['top_jpeg']);actual=restored.predict(row['own_jpeg'],row['top_jpeg']);assert actual==expected;probes.append({'sample_id':row['sample_id'],'decision':actual})
        for split,rset,cache in [('train',rows,features),('development',drows,dfeatures)]:
            metrics,values=evaluate(policy,cache,rset);write(folder/(split+'-metrics.json'),metrics)
            write(folder/(split+'-predictions.json'),[{'id':r['sample_id'],'case_key':r['case_key'],'target':r['target'],'prediction':v} for r,v in zip(rset,values)])
        report['robots'][rid]={'selected':selected,'samples':len(rows),'development_samples':len(drows),'groups':counts,'readback':probes,'parameters':sum(v.numel() for v in policy.parameters()),'trainable_parameters':sum(v.numel() for v in policy.parameters() if v.requires_grad),'training_s':time.monotonic()-begin}
        write(a.out/'report.json',report)
    report['complete']=True;report['wall_s']=time.monotonic()-started;report['artifacts']={str(f.relative_to(a.out)):sha(f) for f in sorted(a.out.rglob('*')) if f.is_file() and f.name!='report.json'};write(a.out/'report.json',report)
if __name__=='__main__':main()
