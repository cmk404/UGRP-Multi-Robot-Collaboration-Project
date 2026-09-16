from harness.recovery_commands import issued_command

def test_calibration_uses_only_predicted_commands_and_preserves_ready():
 d={'ready':False,'forward':.0046189,'left':.0003,'turn':-.0056}
 assert issued_command(d,'calibrated')=={'forward':.005,'left':0.,'turn':-.006}
 assert issued_command(d)=={k:d[k] for k in ('forward','left','turn')}
 assert issued_command({**d,'ready':True},'calibrated')==dict.fromkeys(('forward','left','turn'),0.)
 assert issued_command({'ready':False,'forward':-.04,'left':-.008,'turn':.08},'calibrated')=={'forward':-.04,'left':-.012,'turn':.08}
