"""Floor/obstacle measurements from RGB pixels and own calibrated camera pose."""
import base64
import cv2
import numpy as np
from sim.masterpi_camera_profile import CAMERA_FX_PX, CAMERA_FY_PX, CAMERA_CX_PX, CAMERA_CY_PX, CAMERA_FISHEYE_D

K=np.array([[CAMERA_FX_PX,0,CAMERA_CX_PX],[0,CAMERA_FY_PX,CAMERA_CY_PX],[0,0,1.]])
D=np.array(CAMERA_FISHEYE_D)


def decode(image):
    result=cv2.imdecode(np.frombuffer(base64.b64decode(image),np.uint8),cv2.IMREAD_COLOR)
    if result is None:raise ValueError('INVALID_CAMERA_JPEG')
    return result


def floor_point(pixel, pose, camera_to_base):
    ray=cv2.fisheye.undistortPoints(np.array(pixel,dtype=float).reshape(1,1,2),K,D).reshape(2)
    origin=np.asarray(camera_to_base([0.,0.,0.],pose),dtype=float)
    direction=np.asarray(camera_to_base([ray[0],ray[1],1.],pose),dtype=float)-origin
    if direction[2]>=-.025:return None
    scale=-origin[2]/direction[2]
    result=origin+scale*direction
    if scale<=0 or np.linalg.norm(result[:2])>5:return None
    return result.tolist()


def observe_zone(image, zone, pose, camera_to_base):
    hsv=cv2.cvtColor(decode(image),cv2.COLOR_BGR2HSV)
    ranges={'A':((98,90,30),(130,255,255)), 'B':((38,85,25),(85,255,255)), 'C':((20,90,45),(38,255,255))}
    low,high=ranges[zone]
    mask=cv2.inRange(hsv,np.array(low),np.array(high))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=[c for c in contours if cv2.contourArea(c)>70]
    if not contours:return {'visible':False,'source':'own_rgb_floor_color'}
    c=max(contours,key=cv2.contourArea);mom=cv2.moments(c)
    pixel=[mom['m10']/mom['m00'],mom['m01']/mom['m00']]
    base=floor_point(pixel,pose,camera_to_base)
    return {'visible':base is not None,'pixel':pixel,'estimated_base_m':base,'area_px':cv2.contourArea(c),'source':'own_rgb_color_floor_projection'}
