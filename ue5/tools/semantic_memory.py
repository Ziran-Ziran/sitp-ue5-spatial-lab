"""Goal selection from observed entities only, under explicit engine supervision.

No world catalog, truth, images or path queries are accepted here. TTL is measured
in engine simulated seconds. A new instance is required for every run.
"""
import math


CONDITION='engine_local_visibility_supervision'
CATEGORIES={'chair','lamp'}


def finite(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError('expected finite number')
    return value


def position(value):
    if not isinstance(value,list) or len(value)!=3:
        raise ValueError('expected three coordinates in centimeters')
    return [finite(x) for x in value]


def validate_entities(entities):
    if not isinstance(entities,list):
        raise ValueError('entities must be a list')
    result={}
    for entity in entities:
        if set(entity)!={'entity_id','category','position_cm','visible_sample_count'}:
            raise ValueError('only observed entity fields are allowed')
        key=entity['entity_id']
        if not isinstance(key,str) or not key or len(key)>256 or key in result:
            raise ValueError('invalid or duplicate observed entity')
        samples=entity['visible_sample_count']
        if entity['category'] not in CATEGORIES or isinstance(samples,bool) or not isinstance(samples,int) or not 1<=samples<=3:
            raise ValueError('unsupported engine-supervised category or visibility count')
        result[key]=dict(entity_id=key,category=entity['category'],position_cm=position(entity['position_cm']),
                         visible_sample_count=samples)
    return result


class SemanticMemory:
    def __init__(self,run_id,ttl_s,*,condition):
        if condition!=CONDITION:
            raise ValueError('engine labels/poses require the explicit local-supervision condition')
        if not isinstance(run_id,str) or not run_id or not 0<=finite(ttl_s)<=120:
            raise ValueError('invalid run or bounded TTL')
        self.run_id=run_id
        self.ttl_s=ttl_s
        self.frame_id=-1
        self.time=None
        self.observer=None
        self.entities={}

    def update(self,frame):
        allowed={'record_type','run_id','frame_id','t_sim_s','t_monotonic_s','observer_position_cm','sensor_eye_cm','entities'}
        if set(frame)-allowed or frame.get('record_type')!='frame' or frame.get('run_id')!=self.run_id:
            raise ValueError('unexpected frame fields or another run')
        index=frame['frame_id']
        timestamp=finite(frame['t_sim_s'])
        if (isinstance(index,bool) or not isinstance(index,int) or index!=self.frame_id+1 or timestamp<0
                or (self.time is not None and timestamp<=self.time)):
            raise ValueError('frame IDs must be contiguous and simulated timestamps strictly increasing')
        observer=position(frame['observer_position_cm'])
        observed=validate_entities(frame['entities'])
        # Complete validation first: malformed frames cannot age/prune the memory.
        retained={key:dict(value) for key,value in self.entities.items()
                  if timestamp-value['last_seen_sim_s']<=self.ttl_s}
        for key,entity in observed.items():
            if key in retained and retained[key]['category']!=entity['category']:
                raise ValueError('engine-supervised entity changed category within a run')
            retained[key]=dict(entity,last_seen_sim_s=timestamp,source_frame_id=index)
        self.entities=retained
        self.time=timestamp
        self.frame_id=index
        self.observer=observer

    def select(self,category):
        candidates=[e for e in self.entities.values() if e['category']==category]
        if not candidates:
            return None
        entity=min(candidates,key=lambda e:(math.dist(self.observer,e['position_cm']),e['entity_id']))
        return dict(entity_id=entity['entity_id'],category=entity['category'],position_cm=list(entity['position_cm']),
                    age_sim_s=self.time-entity['last_seen_sim_s'],source_frame_id=entity['source_frame_id'],
                    source='engine_visibility_supervision')
