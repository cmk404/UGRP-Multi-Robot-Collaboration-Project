"""Plot output-only height measurements from the preserved raw evaluations."""
from pathlib import Path
import argparse,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
 fig,axes=plt.subplots(1,2,figsize=(11.5,4.5),sharey=True);data=[]
 for ax,mode in zip(axes,('stationary','shuttle')):
  for label,root,color in [('Baseline',a.baseline,'#d35b47'),('Retention fix',a.raw,'#267c9b')]:
   raw=root/mode;report=json.loads((raw/'result.json').read_text());rows=[json.loads(x) for x in (raw/'evaluation-only.jsonl').read_text().splitlines()];carry=[r for r in rows if r['phase'] in ('carry','carry_stop')]
   t=[r['sim_time_s']-report['endurance_start_s'] for r in carry];drop=[1000*(carry[0]['position_m'][2]-r['position_m'][2]) for r in carry]
   ax.plot(t,drop,color=color,lw=2,label=label);data.append({'mode':mode,'label':label,'raw':str(raw.resolve()),'time_s':t,'drop_mm':drop})
  ax.axhline(10,color='#666666',ls='--',lw=1,label='10 mm limit');ax.set(xlim=(0,302),ylim=(-1,32),title=mode.capitalize(),xlabel='Carry elapsed time (s)');ax.grid(alpha=.16)
 axes[0].set_ylabel('Beam height decrease (mm)');axes[1].legend(loc='upper left',frameon=False)
 fig.suptitle('Fixed-start simulation: retention during 300 s carry',fontweight='bold');fig.text(.5,.015,'Output-only evaluation | original camera placement | weld OFF | zero LLM calls',ha='center',fontsize=9,color='#555555');fig.tight_layout(rect=(0,.04,1,1));fig.savefig(a.out/'height-retention.png',dpi=180)
 (a.out/'height-plot-data.json').write_text(json.dumps(data)+'\n')
if __name__=='__main__':main()
