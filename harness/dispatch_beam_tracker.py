"""Temporal carried-shaft identity from raw TOP RGB, never issued motion/truth."""
import copy,math
import numpy as np
from harness.camera_beam_features import carried_beam_scans

class CarriedBeamTracker:
    def __init__(self):self.previous=None

    @staticmethod
    def _axis(b):
        v=(np.array(b['endpoints'][1])-b['endpoints'][0])*b['image_size']
        return v/np.linalg.norm(v)

    @classmethod
    def same_shaft(cls,a,b):
        axis=cls._axis(a);other=cls._axis(b)
        delta=(np.array(b['center'])-a['center'])*a['image_size']
        return (abs(float(axis@other))>math.cos(math.radians(5))
            and abs(float(delta@[-axis[1],axis[0]]))<5
            and abs(float(delta@axis))<.22*min(a['length_px'],b['length_px']))

    def observe(self,jpeg):
        from harness.dispatch_skill_binding import beam_feature
        anchor=self.previous or beam_feature(jpeg,hue_upper=35)
        rows=[]
        for saturation, candidates in zip(range(105,191,5), carried_beam_scans(jpeg)):
            for b in candidates:
                movement=np.linalg.norm((np.array(b['center'])-anchor['center'])*b['image_size'])
                aligned=abs(float(self._axis(b)@self._axis(anchor)))>=math.cos(math.radians(15))
                if (not b['touches_border'] and 65<=b['length_px']<=130 and b['width_px']<=25
                    and b['length_px']/b['width_px']>=3.5 and movement<=24 and aligned):
                    rows.append((saturation,b))
        supported=[]
        for saturation,b in rows:
            group=[(s,c) for s,c in rows if self.same_shaft(b,c)]
            count=len({s for s,_ in group})
            # Sample narrow usable contrast bands without reducing the required
            # contrast range: adjacent cuts alone cannot establish a shaft.
            contrast_span=max(s for s,_ in group)-min(s for s,_ in group)
            if count>=3 and contrast_span>=20:supported.append((count,saturation,b,group))
        if not supported:raise ValueError('carried shaft lacks consistent RGB support')
        best=max(supported,key=lambda row:(row[0],-np.linalg.norm((np.array(row[2]['center'])-anchor['center'])*anchor['image_size'])))
        if any(not self.same_shaft(best[2],r[2]) and r[0]>=best[0]-1 for r in supported):
            raise ValueError('carried shaft identity ambiguous in RGB')
        # Choose an actual segmented shaft close to the consensus centre. Do
        # not fabricate unseen endpoints; visible-fragment uncertainty is kept.
        median=np.median([b['center'] for _,b in best[3]],axis=0)
        saturation,b=min(best[3],key=lambda row:np.linalg.norm((np.array(row[1]['center'])-median)*anchor['image_size']))
        feature=copy.deepcopy(b)
        feature['tracking']={'method':'temporal stable-width shaft consensus','threshold_support':best[0],
            'selected_saturation':saturation,'prior_center':anchor['center'],
            'uses_issued_motion':False}
        self.previous=copy.deepcopy(feature)
        return feature
