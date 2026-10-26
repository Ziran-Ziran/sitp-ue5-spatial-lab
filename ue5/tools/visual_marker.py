"""Explicit controlled color-marker categories from RGB + optical depth.

Fixed HSV/height rules, no learned furniture semantics or engine metadata.
"""
import cv2
import numpy as np
from local_depth_grid import unproject
from marker_extent import VARIANTS,inspect_extent

CATEGORIES={'red_marker','blue_marker'}


def detect_markers(color,depth,calibration,*,perception_variant=None):
    if perception_variant is not None and perception_variant not in VARIANTS:raise ValueError('explicit controlled perception variant required')
    # Validate metric depth/intrinsics even when no color region is found.
    unproject(depth,calibration,eye_height_m=1.5)
    if (not isinstance(color,np.ndarray) or color.dtype!=np.uint8
            or color.shape!=(calibration['h'],calibration['w'],3)):
        raise ValueError('marker RGB must be uint8 BGR with calibrated dimensions')
    hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV);hue,saturation,value=cv2.split(hsv)
    colorful=(saturation>=120)&(value>=50)
    masks={'red_marker':colorful&((hue<=10)|(hue>=170)),
           'blue_marker':colorful&(hue>=100)&(hue<=130)}
    regions=[];label_images={}
    for category,mask in masks.items():
        count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity=8)
        label_images[category]=labels
        for label in range(1,count):
            x,y,w,h,area=(int(v) for v in stats[label]);regions.append((category,label,area,[x,y,w,h]))
    regions.sort(key=lambda region:(-region[2],region[0],region[1]));detections=[];rejected=[]
    for category,label,area,bbox in regions[:64]:
        item=dict(category=category,component_id=label,pixels=area,bbox_xywh=bbox)
        reason=None;mask=label_images[category]==label
        valid=mask&(depth>0)&(depth<65535);fraction=float(np.count_nonzero(valid)/area)
        item['depth_valid_fraction']=fraction
        if area<9:reason='too_few_pixels'
        elif fraction<.5:reason='insufficient_valid_depth'
        else:
            values=depth[valid].astype(float)/1000
            spread=float(np.percentile(values,90)-np.percentile(values,10));item['depth_spread_m']=spread
            if spread>.5:reason='depth_discontinuity'
            else:
                points=unproject(np.where(valid,depth,0).astype(np.uint16),calibration,eye_height_m=1.5)
                center=np.median(points,axis=0).tolist()
                if not .4<=center[2]<=1.6:reason='outside_marker_height_prior'
                if reason is None and perception_variant=='known_box_extent_v1':
                    item['extent_prior']=inspect_extent(points)
                    if not item['extent_prior']['accepted']:reason='outside_known_marker_box_extent'
                if reason is None:
                    item['position_right_forward_up_m']=center
                    item['planar_range_m']=float(np.linalg.norm(center[:2]));detections.append(item)
        if reason:rejected.append(dict(item,reason=reason))
    detections.sort(key=lambda item:(item['planar_range_m'],item['category'],item['component_id']))
    result=dict(schema_version=1,condition='fixed_hsv_visual_marker_detector',detections=detections,rejected=rejected,
        components_total=len(regions),components_processed=min(64,len(regions)),component_budget_exhausted=len(regions)>64,
        coordinates='local right forward up meters; explicit level camera and1.5m eye-height prior',
        interpretation='controlled red/blue marker categories and depth position only; not furniture recognition or calibrated semantic confidence')
    if perception_variant=='known_box_extent_v1':
        result.update(schema_version=2,condition='fixed_hsv_known_box_extent_visual_marker_detector',perception_variant=perception_variant,
            interpretation='fixed color plus known controlled-box upper extent prior; not general object identity or semantic confidence')
    return result
