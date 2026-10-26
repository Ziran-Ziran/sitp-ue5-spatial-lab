"""Predeclared primitive fixtures; construction/evaluation, never perception input."""
import math


def cases():
    definitions=[('flat',0,None,False),('up5',5,None,False),('down5',-5,None,False),
        ('up10_boundary',10,None,False),('down10_boundary',-10,None,False),
        ('up15_contract',15,None,False),('down15_contract',-15,None,False),
        ('low170',0,170,True),('high220',0,220,True),('slope10_low1765',10,176.5,True)]
    return [dict(case_id=name,slope_deg=angle,ceiling_vertical_gap_cm=gap,supplied_reference=reference,
        start_cm=[0,index*2000,0],heading_deg=0,query_right_forward_m=[0,1])
        for index,(name,angle,gap,reference) in enumerate(definitions)]


def floor_transform(case):
    angle=math.radians(case['slope_deg'])
    return [0,case['start_cm'][1],-10/math.cos(angle)],[20,10,.2],case['slope_deg']


def ceiling_transform(case):
    gap=case['ceiling_vertical_gap_cm']
    if gap is None:raise ValueError('explicit ceiling case required')
    angle=math.radians(case['slope_deg'])
    return [115,case['start_cm'][1],gap+10/math.cos(angle)+115*math.tan(angle)],[1/math.cos(angle),10,.2],case['slope_deg']
