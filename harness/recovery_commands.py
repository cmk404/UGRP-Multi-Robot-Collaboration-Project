"""Stateless actuator decoding; no observations or teacher state are accepted."""
import math
AXES=('forward','left','turn')
# Activation thresholds suppress regression noise; minimum magnitudes match the
# physically verified demonstrator. This changes commands, never stop readiness.
FLOORS={'forward':(.003,.005),'left':(.004,.012),'turn':(.004,.006)}

def issued_command(decision, decoder='raw'):
 if decoder not in ('raw','calibrated'):raise ValueError('unknown command decoder')
 if decision['ready']:return dict.fromkeys(AXES,0.)
 command={k:float(decision[k]) for k in AXES}
 if not all(math.isfinite(v) for v in command.values()):raise ValueError('finite commands required')
 if decoder=='calibrated':
  for k,(threshold,minimum) in FLOORS.items():
   v=command[k];command[k]=math.copysign(max(minimum,abs(v)),v) if abs(v)>threshold else 0.
 return command
