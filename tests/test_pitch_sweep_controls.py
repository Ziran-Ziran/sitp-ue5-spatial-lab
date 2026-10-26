"""Ray/plane synthetic expectations for the owned primitive controls."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from pitch_sweep_controls_report import evaluate_control_pixels


def case_view(kind,pitch):
    v,u=np.indices((240,320));x=(u-159.5)/160;down=(v-119.5)/160
    angle=np.deg2rad(pitch);forward=np.cos(angle)-down*np.sin(angle)
    up=-np.sin(angle)-down*np.cos(angle)
    best=np.full((240,320),np.inf)
    floors=[(-5,15,0)] if kind!='drop' else [(-5,1,0),(3,15,0),(1,3,-1)]
    for near,far,height in floors:
        t=np.divide(height-1.5,up,out=np.full_like(up,np.inf),where=abs(up)>1e-12)
        f=t*forward;r=t*x
        keep=(t>0)&(f>=near)&(f<=far)&(abs(r)<=5)
        best=np.where(keep,np.minimum(best,t),best)
    if kind=='wall':
        t=np.divide(1.5,forward,out=np.full_like(forward,np.inf),where=abs(forward)>1e-12)
        keep=(t>0)&(abs(t*x)<=1)&(1.5+t*up>=0)&(1.5+t*up<=2)
        best=np.where(keep,np.minimum(best,t),best)
    best[~np.isfinite(best)|(best>30)]=0
    return np.rint(best*1000).astype(np.uint16)


class PitchSweepControlTests(unittest.TestCase):
    def cases(self):return {case:{pitch:case_view(case,pitch) for pitch in (0,45,90)} for case in ('flat','wall','drop')}

    def test_flat_floor_reveals_start_but_wall_and_drop_cannot_pass(self):
        result=evaluate_control_pixels(self.cases())
        self.assertTrue(result['passed'],result)

    def test_bad_pitch_depth_or_erased_drop_fails_acceptance(self):
        cases=self.cases();cases['flat'][90][:]=0
        self.assertFalse(evaluate_control_pixels(cases)['passed'])
        cases=self.cases();cases['drop']=cases['flat']
        self.assertFalse(evaluate_control_pixels(cases)['passed'])
