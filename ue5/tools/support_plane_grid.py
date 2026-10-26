"""Observed support plane with explicit near-foot priors; no unseen floor fill."""
import math
import numpy as np
from local_depth_grid import GridConfig,points_grid,unproject

MODEL='observed_support_plane_v3'


def valid_points(points,config):
    if (not isinstance(config,GridConfig) or not isinstance(points,np.ndarray) or points.ndim!=2 or points.shape[1]!=3
            or len(points)>250000 or points.dtype.kind not in 'fiu' or not np.isfinite(points).all()):
        raise ValueError('bounded finite observed Nx3 local points and explicit grid required')


def fit_support_plane(points,config):
    valid_points(points,config)
    result=dict(model=MODEL,status='insufficient_seed_cells',seed_radius_m=1,seed_height_band_m=.6,
        inlier_tolerance_m=config.floor_tolerance_m/2,minimum_inlier_fraction=.6,max_slope_deg=45,
        maximum_abs_intercept_m=config.floor_tolerance_m,hypothesis_budget=65,seed_cells=0,
        coefficients_right_forward_up=None,reference='tilt_plane_anchored_to_actual_capsule_feet_height_0')
    seed=points[(np.linalg.norm(points[:,:2],axis=1)<=1)&(np.abs(points[:,2])<=.6)]
    cells=np.floor(seed[:,:2]/config.resolution_m).astype(np.int64)
    _,indices=np.unique(cells,axis=0,return_inverse=True)
    samples=[]
    for index in range(int(indices.max())+1 if len(indices) else 0):
        group=seed[indices==index];order=np.argsort(group[:,2],kind='stable');samples.append(group[order[len(order)//2]])
    if len(samples)>1024:samples=[samples[i] for i in np.linspace(0,len(samples)-1,1024,dtype=int)]
    samples=np.asarray(samples,dtype=float).reshape((-1,3));result['seed_cells']=len(samples)
    if len(samples)<16:return result
    design=np.column_stack((samples[:,:2],np.ones(len(samples))));height=samples[:,2]
    if np.linalg.matrix_rank(design)<3:return dict(result,status='degenerate_seed_geometry')
    rng=np.random.default_rng(0);hypotheses=[np.linalg.lstsq(design,height,rcond=None)[0]]
    for _ in range(64):
        chosen=rng.choice(len(samples),3,replace=False);matrix=design[chosen]
        if np.linalg.matrix_rank(matrix)==3:hypotheses.append(np.linalg.solve(matrix,height[chosen]))
    def bounded(coefficients):
        return (np.isfinite(coefficients).all() and abs(coefficients[2])<=config.floor_tolerance_m
                and np.hypot(*coefficients[:2])<=math.tan(math.radians(45)))
    best=None
    for coefficients in hypotheses:
        if not bounded(coefficients):continue
        residual=np.abs(design@coefficients-height);inliers=residual<=config.floor_tolerance_m/2
        if np.count_nonzero(inliers)<.6*len(samples):continue
        key=(-int(inliers.sum()),float(np.sqrt(np.mean(residual[inliers]**2))),abs(float(coefficients[2])),float(np.hypot(*coefficients[:2])))
        if best is None or key<best[0]:best=(key,coefficients,inliers)
    if best is None:return dict(result,status='no_bounded_consensus')
    _,coefficients,inliers=best
    unconstrained_intercept=float(coefficients[2])
    for _ in range(2):
        if np.linalg.matrix_rank(design[inliers])<3:return dict(result,status='degenerate_inliers')
        # Keep the capsule's actual height origin. Subtracting a fitted negative
        # intercept would otherwise hide a ceiling below its physical top.
        slopes=np.linalg.lstsq(design[inliers,:2],height[inliers],rcond=None)[0]
        coefficients=np.array([*slopes,0.])
        residual=np.abs(design@coefficients-height);inliers=residual<=config.floor_tolerance_m/2
        if not bounded(coefficients) or inliers.sum()<.6*len(samples):return dict(result,status='refinement_rejected')
    support=samples[inliers,:2]
    for right_sign,forward_sign in ((-1,-1),(-1,1),(1,-1),(1,1)):
        if not np.any((support[:,0]*right_sign>=config.resolution_m)&(support[:,1]*forward_sign>=config.resolution_m)):
            return dict(result,status='insufficient_surrounding_support')
    return dict(result,status='supported',coefficients_right_forward_up=coefficients.tolist(),inlier_cells=int(inliers.sum()),
        inlier_fraction=float(inliers.mean()),residual_rmse_m=float(np.sqrt(np.mean(residual[inliers]**2))),
        slope_deg=math.degrees(math.atan(float(np.hypot(*coefficients[:2])))),hypotheses_evaluated=len(hypotheses),
        unconstrained_intercept_m=unconstrained_intercept)


def support_plane_points_grid(points,config):
    fit=fit_support_plane(points,config)
    if fit['status']=='supported':
        coefficients=np.asarray(fit['coefficients_right_forward_up']);relative=points.copy().astype(float)
        relative[:,2]-=points[:,:2]@coefficients[:2]+coefficients[2]
        # The agent remains upright: a sloping reference must not rotate its
        # capsule or turn same-XY residual height into physical head clearance.
        # For every point in the radius-bounding square, reference height can
        # differ by radius*(|a|+|b|). Add the accepted floor band as uncertainty.
        # Clamp only these extra overhead hazards into the existing body band;
        # floor, negative, XY sampling and the original grid config stay intact.
        overhead_allowance=config.floor_tolerance_m+config.radius_m*float(np.abs(coefficients[:2]).sum())
        upper=config.body_height_m+overhead_allowance
        overhead=(relative[:,2]>config.body_height_m)&(relative[:,2]<=upper)
        relative[overhead,2]=config.body_height_m
        grid=points_grid(relative,config,condition='depth_geometry_observed_local_support_plane')
        fit.update(upright_obstacle_residual_ceiling_m=upper,
            upright_overhead_allowance_m=overhead_allowance,additional_overhead_samples=int(overhead.sum()),
            clearance_bound='body_height + floor_tolerance + radius*(abs(right_slope)+abs(forward_slope))')
    else:
        inside=((points[:,0]>=config.right_min_m)&(points[:,0]<config.right_max_m)
            &(points[:,1]>=config.forward_min_m)&(points[:,1]<config.forward_max_m))
        grid=dict(config=config,states=np.full(config.shape,-1,dtype=np.int8),free=np.zeros(config.shape,dtype=bool),
            valid_depth_points=len(points),in_bounds_points=int(inside.sum()),floor_samples=0,obstacle_samples=0,negative_samples=0,
            unclassified_samples=len(points),condition='depth_geometry_unsupported_local_plane')
    grid.update(ground_model=MODEL,support_plane=fit,
        interpretation='depth-only near-foot plane estimate and residual classification of observed points; unknown remains blocked; no engine geometry, terrain curvature or physics guarantee')
    return grid


def support_plane_grid(views,calibration,config):
    if not isinstance(config,GridConfig) or not isinstance(views,(list,tuple)) or not 1<=len(views)<=3:raise ValueError('explicit grid and one to three calibrated pitch views required')
    pitches=[p for p,_ in views]
    if any(type(p) not in (int,float) or p not in (0,45,90) for p in pitches) or len(set(pitches))!=len(pitches):
        raise ValueError('unique downward pitches 0/45/90 required')
    points=np.concatenate([unproject(depth,calibration,eye_height_m=config.eye_height_m,pitch_down_deg=p) for p,depth in views])
    return support_plane_points_grid(points,config)
