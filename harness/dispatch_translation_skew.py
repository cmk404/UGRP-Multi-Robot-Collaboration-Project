"""Current RGB shaft-centerline measurement for the nonrotating carry skill."""
import math
import cv2
import numpy as np
from harness.camera_goal_transport import decode


def translation_skew(jpeg, beam):
    """Exclude shaded edge lobes before measuring bottom-minus-top skew.

    The tracked visible shaft supplies the search region, not its noisy PCA
    direction as the final answer. Fit current central cross-section medians.
    No temporal smoothing, issued movement or referee pose supplies the angle.
    """
    frame=decode(jpeg);hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    endpoints=np.array(beam['endpoints'])*beam['image_size']
    axis=endpoints[1]-endpoints[0];axis/=np.linalg.norm(axis)
    if axis[1]<0:axis=-axis
    normal=np.array([-axis[1],axis[0]])
    center=np.array(beam['center'])*beam['image_size']
    radius=math.ceil(beam['length_px']*.65);cx,cy=np.rint(center).astype(int)
    x0,x1=max(0,cx-radius),min(frame.shape[1],cx+radius+1)
    y0,y1=max(0,cy-radius),min(frame.shape[0],cy+radius+1)
    # The weakly saturated yellow apron can enter this shaft-sized window.
    # Retain the saturated cargo face rather than fitting those floor pixels.
    mask=cv2.inRange(hsv[y0:y1,x0:x1],np.array([3,150,45],np.uint8),np.array([35,255,255],np.uint8))
    yy,xx=np.where(mask);points=np.column_stack((xx+x0,yy+y0))
    axial=(points-center)@axis;lateral=(points-center)@normal
    selected=(abs(axial)<beam['length_px']*.35)&(abs(lateral)<beam['width_px']*.75)
    # Half-pixel centres on an exactly vertical shaft must still produce one
    # section per image row. Banker's rounding aliases alternate rows.
    indices=np.floor(axial).astype(int);sections=[]
    for index in sorted(set(indices[selected])):
        values=lateral[(indices==index)&selected]
        if len(values)>=beam['width_px']*.6:
            sections.append([index,float(np.median(values))])
    if len(sections)<max(20,beam['length_px']*.45):
        raise ValueError('translation shaft centerline lacks current RGB support')
    points=np.array(sections,dtype=np.float32)
    vx,vy,x,y=cv2.fitLine(points,cv2.DIST_L1,0,.01,.01).ravel()
    if abs(vx)<.5:raise ValueError('translation shaft centerline axis ambiguous')
    residual=abs(points[:,1]-(y+(points[:,0]-x)*vy/vx))
    if np.ptp(points[:,0])<beam['length_px']*.5 or np.quantile(residual,.95)>2.:
        raise ValueError('translation shaft centerline inconsistent in RGB')
    direction=axis+normal*vy/vx;direction/=np.linalg.norm(direction)
    if abs(float(direction@axis))<math.cos(math.radians(8)) or direction[1]<.8:
        raise ValueError('translation shaft centerline disagrees with tracked shaft')
    skew=float(direction[0]*beam['length_px'])
    return skew,{'method':'current central RGB cross-section median line',
        'skew_px':skew,'direction_xy':direction.tolist(),'supported_sections':len(sections),
        'support_span_px':float(np.ptp(points[:,0])),
        'residual_p95_px':float(np.quantile(residual,.95)),
        'tracked_visible_length_px':beam['length_px'],'min_saturation':150,'uses_issued_motion':False}
