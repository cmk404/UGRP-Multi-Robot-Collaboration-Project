from pathlib import Path
import json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
report=json.loads(Path(sys.argv[1]).read_text());out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
variant=sys.argv[3] if len(sys.argv)>3 else 'audited'
assert variant in ('audited','original')
case_key='per_case_audited' if variant=='audited' else 'per_case'
success_key='audited_successes' if variant=='audited' else 'successes'
suffix='' if variant=='audited' else '-original'
arms=['rule-numeric','jev-numeric','jev-semantic','gemini-numeric','gemini-semantic'];s=report['summary']['new'];cases=sorted(s[arms[0]]['per_case'])
a=np.array([[s[arm][case_key][case] for arm in arms] for case in cases])
fig,ax=plt.subplots(figsize=(10,8));fig.patch.set_facecolor('#f8fafc')
cmap=ListedColormap(['#fecaca','#fde68a','#d9f99d','#86efac'])
ax.imshow(a,cmap=cmap,norm=BoundaryNorm([-.5,.5,1.5,2.5,3.5],4),aspect='auto')
ax.set_xticks(range(5),['Rule\nnumeric','Jev\nnumeric','Jev\nsemantic','Gemini\nnumeric','Gemini\nsemantic']);ax.set_yticks(range(12),cases)
for i in range(12):
 for j in range(5):ax.text(j,i,f'{a[i,j]}/3',ha='center',va='center',fontsize=12,color='#172033')
ax.set_xticks(np.arange(-.5,5,1),minor=True);ax.set_yticks(np.arange(-.5,12,1),minor=True);ax.grid(which='minor',color='white',linewidth=3);ax.tick_params(which='both',length=0);ax.tick_params(axis='x',labelsize=11,pad=10)
for spine in ax.spines.values():spine.set_visible(False)
fig.suptitle(f'Closed-loop approach: {variant} outcomes',fontsize=18,x=.52,y=.98)
fig.text(.52,.925,'12 new starting poses | 3 API repeats each | fixed map and box',ha='center',fontsize=11)
fig.text(.52,.075,'Totals: '+ '  |  '.join(f'{arm}: {s[arm][success_key]}/36' for arm in arms),ha='center',fontsize=9)
fig.text(.52,.035,('Audited: 1 ns float tolerance for the 0.350 s evaluation only; original verdicts retained.\n' if variant=='audited' else 'Original frozen-run verdicts, including floating-point evaluation failures.\n')+'Same RGB observer and 7 actions. Paused SIM. Not grasp/carry or hardware validation.',ha='center',fontsize=10,color='#475569')
fig.subplots_adjust(top=.89,bottom=.17,left=.12,right=.96)
fig.savefig(out/f'success-matrix{suffix}.png',dpi=160);fig.savefig(out/f'success-matrix{suffix}.pdf');print(out/f'success-matrix{suffix}.png')
