"""Independent pixel/depth controls for synchronous static-pose replay rendering."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def validate_pair(color,depth):
    if (color is None or depth is None or color.dtype!=np.uint8 or color.shape!=(240,320,3)
            or depth.dtype!=np.uint16 or depth.shape!=(240,320)):
        raise ValueError('expected 320x240 uint8 BGR RGB decode and uint16 single-channel millimeter depth')
    if np.any((depth>30000)&(depth!=65535)):raise ValueError('depth exceeds declared valid range')


def validate_controls(frames):
    if set(frames)!={'pose-a','pose-b','pose-a-repeat','empty-backward','exposure-dark'}:
        raise ValueError('all asymmetric/translation/repeat/empty controls required')
    for color,depth in frames.values():validate_pair(color,depth)
    color,a=frames['pose-a']
    b=frames['pose-b'][1]
    repeat_color,repeat=frames['pose-a-repeat']
    empty_color,empty=frames['empty-backward']
    dark,dark_depth=frames['exposure-dark']
    red=color[:,:,2].astype(float)
    green=color[:,:,1].astype(float)
    blue=color[:,:,0].astype(float)
    red_mask=(red>30)&(red>green*1.5)&(red>blue*1.5)
    green_mask=(green>30)&(green>red*1.5)&(green>blue*1.5)
    checks=dict(
        plane_axis_depth_a=abs(int(a[119,159])-10000)<=5,
        plane_off_axis_depth=abs(int(a[230,10])-10000)<=5,
        translated_pose_depth=abs(int(b[119,159])-8000)<=5,
        foreground_depth=abs(int(a[54,78])-4900)<=5,
        upper_left_is_red=bool(red_mask[54,78]),
        lower_right_is_green=bool(green_mask[185,241]),
        repeat_depth_matches=bool(np.max(np.abs(a.astype(float)-repeat.astype(float)))<=1),
        repeat_color_matches=bool(np.mean(np.abs(color.astype(float)-repeat_color.astype(float)))<=1),
        empty_depth_is_invalid_zero=bool(np.all(empty==0)),
        exposure_changes_rgb=bool(float(dark.mean())<float(color.mean())*.1),
        exposure_does_not_change_depth=bool(np.max(np.abs(a.astype(float)-dark_depth.astype(float)))<=1),
        no_saturated_valid_depth=bool(all(not np.any(depth==65535) for _,depth in frames.values())))
    expected_center=[159.5+160*(-250)/490,119.5-160*200/490]
    ys,xs=np.nonzero(red_mask)
    centroid=[float(xs.mean()),float(ys.mean())] if len(xs) else None
    extent=[int(xs.max()-xs.min()+1),int(ys.max()-ys.min()+1)] if len(xs) else None
    checks['principal_point_and_axes']=bool(centroid and np.linalg.norm(np.array(centroid)-expected_center)<=3)
    checks['focal_length_extent']=bool(extent and all(abs(value-200*160/490)<=4 for value in extent))
    return dict(evidence_kind='synthetic_rgbd_pixel_and_depth_acceptance',passed=all(checks.values()),
        checks={key:bool(value) for key,value in checks.items()},red_centroid_xy=centroid,red_extent_wh=extent,
        expected_red_center_xy=expected_center,
        measured_depth_mm=dict(plane=int(a[119,159]),off_axis=int(a[230,10]),translated=int(b[119,159]),foreground=int(a[54,78])),
        interpretation='synthetic calibration/readback controls; no live sampling, recognition or SLAM tested')


def read_controls(directory):
    record=json.loads((directory/'render.json').read_text(encoding='utf-8-sig'))
    frames={}
    for row in record['frames']:
        if not row.get('captured'):raise ValueError('native render incomplete')
        color=cv2.imread(str(directory/row['color_file']),cv2.IMREAD_COLOR)
        depth=cv2.imread(str(directory/row['depth_file']),cv2.IMREAD_UNCHANGED)
        frames[row['name']]=(color,depth)
    return frames


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    report=validate_controls(read_controls(args.directory))
    with args.out.open('x',encoding='utf-8') as output:json.dump(report,output,indent=2)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 2)
