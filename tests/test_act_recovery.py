import math
from scripts.recovery_teacher import command,heldout_region,score_alignment

def test_recovery_has_signed_actions_and_heading_priority():
 assert command({'x':-.01,'y':0,'yaw':0})['forward']<0
 assert command({'x':.2,'y':-.02,'yaw':0})['left']<0
 a=command({'x':.2,'y':.02,'yaw':-.1});assert a['turn']<0 and a['forward']==a['left']==0
 assert command(dict(x=0,y=0,yaw=0))['ready']

def test_combination_screen():
 assert heldout_region(dict(x=.4,y=0,yaw=.1))
 assert heldout_region(dict(x=-.01,y=0,yaw=.1))
 assert heldout_region(dict(x=.2,y=.02,yaw=.1))
 assert not heldout_region(dict(x=.2,y=0,yaw=.1))

def test_alignment_requires_stationary_window():
 goals={'r1':[0,0,0]}
 rows=[{'sim_time_s':i*.1,'phase':'approach_stop_dwell','bases':{'r1':[0,0,0]},'base_yaw_rad':{'r1':0}} for i in range(5)]
 assert score_alignment(rows,goals,.4)['success']
 assert not score_alignment(rows[1:],goals,.4)['success']
 rows[-1]['bases']['r1'][0]=.003
 assert not score_alignment(rows,goals,.4)['success'] # tolerance met, movement not stable
