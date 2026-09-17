"""Observer-only plots and selected video evidence; no actor inputs."""
from pathlib import Path
import json,math,argparse,subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 p=argparse.ArgumentParser();p.add_argument('raw',type=Path);p.add_argument('out',type=Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 fig,axs=plt.subplots(1,2,figsize=(12,4.4),sharey=True);evidence=[]
 for ax,mode in zip(axs,('stationary','shuttle')):
  raw=a.raw/mode;r=json.loads((raw/'result.json').read_text());rows=[json.loads(x) for x in (raw/'evaluation-only.jsonl').read_text().splitlines()];c=[x for x in rows if x['phase'] in ('carry','carry_stop')];start=r['endurance_start_s'];elapsed=[x['sim_time_s']-start for x in c]
  drop=np.array([c[0]['position_m'][2]-x['position_m'][2] for x in c])*1000
  fingers=np.array([np.mean([c[0]['finger_centers_m'][rid][2]-x['finger_centers_m'][rid][2] for rid in ('r1','r3')]) for x in c])*1000
  ax.plot(elapsed,drop,color='#d94d42',lw=2,label='Beam descent');ax.plot(elapsed,fingers,color='#267da5',lw=2,label='Mean finger descent');ax.axhline(10,color='#777',ls='--',lw=1,label='10 mm retention limit')
  ax.set(title=f'{mode.capitalize()} | stopped at {elapsed[-1]:.2f} s',xlabel='Seconds after endurance start',xlim=(0,100),ylim=(-10,80));ax.grid(alpha=.16);ax.legend(loc='upper left',fontsize=9)
  points=[]
  for t in (0,30,60,80,90,elapsed[-1]):
   i=min(range(len(c)),key=lambda i:abs(elapsed[i]-t));points.append({'elapsed_s':elapsed[i],'beam_drop_mm':float(drop[i]),'mean_finger_drop_mm':float(fingers[i]),'bilateral':all(v['bilateral'] for v in c[i]['contacts'].values()),'tilt_deg':c[i]['tilt_deg']})
  centers=np.array([np.mean([x['bases'][rid][:2] for rid in ('r1','r3')],axis=0) for x in c]);evidence.append({'mode':mode,'source_sha':r['source_sha'],'endurance_start_s':start,'points':points,'actual_center_x_span_m':float(np.ptp(centers[:,0])),'first_height_limit_exceeded_s':next((elapsed[i] for i,v in enumerate(drop) if v>10),None)})
 axs[0].set_ylabel('Downward displacement from first carry sample (mm)');fig.suptitle('Long-hold failure: the beam slides while finger height stays nearly fixed',fontsize=13);fig.tight_layout();fig.savefig(a.out/'beam-and-finger-descent.png',dpi=170);plt.close(fig)
 (a.out/'plot-data.json').write_text(json.dumps(evidence,indent=2)+'\n')
 print(json.dumps(evidence,indent=2))
if __name__=='__main__':main()
