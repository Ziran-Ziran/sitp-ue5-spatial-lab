"""Known controlled-box upper extent prior; no minimum size or completion."""
import math
import numpy as np

VARIANTS=('hsv_depth_v1','known_box_extent_v1')
BOX_DIMENSIONS_M=(.2,.2,.6)
QUANTIZATION_ALLOWANCE_M=.002


def inspect_extent(points):
    if (not isinstance(points,np.ndarray) or points.ndim!=2 or points.shape[1]!=3
            or not 1<=len(points)<=76800 or points.dtype.kind not in 'fiu' or not np.isfinite(points).all()):
        raise ValueError('bounded nonempty finite observed RFU component points required')
    horizontal=math.sqrt(sum(value*value for value in BOX_DIMENSIONS_M[:2]))+QUANTIZATION_ALLOWANCE_M
    maximum=np.array([horizontal,horizontal,BOX_DIMENSIONS_M[2]+QUANTIZATION_ALLOWANCE_M])
    spans=np.ptp(points.astype(float),axis=0);violating=np.flatnonzero(spans>maximum).tolist()
    return dict(prior='known_fixture_upright_box_extent',box_dimensions_m=list(BOX_DIMENSIONS_M),
        depth_quantization_allowance_m=QUANTIZATION_ALLOWANCE_M,maximum_spans_right_forward_up_m=maximum.tolist(),
        observed_spans_right_forward_up_m=spans.tolist(),observed_points=len(points),violating_axes=violating,
        accepted=not violating,
        interpretation='necessary observed extent upper bound for a known controlled marker; not object identity, confidence, or unseen completion')
