"""Single observed target goal: legacy lookahead or existing execution horizon."""
import math
from local_depth_grid import number
from semantic_memory import position

POLICIES=('fixed_three_meters','execution_prefix_one_meter')
EXECUTION_PREFIX_M=1


def observed_goal(point,distance,policy):
    r,f,_=position(point);number(distance)
    if policy not in POLICIES or distance<=1.25 or not math.isclose(math.hypot(r,f),distance,rel_tol=1e-9,abs_tol=1e-9):
        raise ValueError('observed unreached marker range and explicit horizon policy required')
    travel=min(3 if policy==POLICIES[0] else EXECUTION_PREFIX_M,distance-.9)
    return [r*travel/distance,f*travel/distance]
