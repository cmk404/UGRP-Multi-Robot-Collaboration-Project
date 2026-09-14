import json,math,tempfile,unittest
from pathlib import Path
from scripts.collect_camera_varied_start_teacher import load_cases,teacher_label,clipped,actor_and_label

class VariedTeacherContractTests(unittest.TestCase):
 def test_load_cases_requires_safe_unique_strict_poses(self):
  good={'cases':[{'case_id':'c-1','start_poses':{'r1':{'distance_m':.3,'lateral_m':.02,'yaw_deg':5},'r3':{'distance_m':.25,'lateral_m':-.02,'yaw_deg':-5}}}]}
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'c.json';p.write_text(json.dumps(good));self.assertEqual(load_cases(p)[0]['case_id'],'c-1')
   for bad in [[],[good['cases'][0],good['cases'][0]],[{'case_id':'../x','start_poses':good['cases'][0]['start_poses']}]]:
    p.write_text(json.dumps({'cases':bad}))
    with self.assertRaises(ValueError):load_cases(p)
 def test_teacher_labels_follow_exact_gains_bounds_and_tolerances(self):
  fixture={'r1':[.58,-2.325,.03]};
  self.assertEqual(teacher_label('yaw','r1',[0,0,0],[0,0,.1],fixture)['command'],-.03)
  self.assertAlmostEqual(teacher_label('lateral','r1',[0,-2.425,0],[0,0,0],fixture)['command'],.035)
  self.assertAlmostEqual(teacher_label('forward','r1',[0,-2.325,0],[0,0,0],fixture)['command'],.116)
  self.assertTrue(teacher_label('yaw','r1',[0,0,0],[0,0,.003],fixture)['ready'])
  self.assertEqual(clipped(.001,-.06,.06,.01),.01)
 def test_actor_record_contains_rgb_only_and_truth_is_separate(self):
  class C:
   def base_xyz(self):return (.3,-2.4,.03)
   def base_rpy(self):return (0.,0.,.1)
  class W:controllers={'r1':C(),'r3':C()}
  class S:
   world=W()
   def capture(self,tag):
    rec={'path':'rgb/x.jpg','sha256':'a'*64}
    return {r:{'own_rgb':rec,'shared_top_rgb':rec} for r in ('r1','r3')}
  actor=[];labels=[];fixture={r:[.58,-2.325,.03] for r in ('r1','r3')};actor_and_label(S(),'c','yaw',0,fixture,actor,labels)
  self.assertEqual(set(actor[0]),{'id','case_id','robot_id','stage','observations'})
  self.assertNotIn('teacher_truth',json.dumps(actor[0]));self.assertIn('teacher_truth',labels[0])
