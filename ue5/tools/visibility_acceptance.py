"""Synthetic ray-supervision controls, separate from visual semantic accuracy."""
from semantic_observation_stream import iter_validated


def validate_synthetic_visibility(fixture,records):
    visible=fixture['semantic_controls']['visible_chair']
    hidden=fixture['semantic_controls']['occluded_lamp']
    first=None
    later_hidden=None
    header=None
    frames=0
    categories_valid=True
    for row in iter_validated(records):
        if row['record_type']=='stream': header=row
        if row['record_type']!='frame' or row['run_id']!='detour': continue
        ids={entity['entity_id'] for entity in row['entities']}
        for entity in row['entities']:
            expected='chair' if entity['entity_id']==visible else 'lamp' if entity['entity_id']==hidden else None
            if expected and entity['category']!=expected: categories_valid=False
        if first is None: first=ids
        if hidden in ids and row['t_sim_s']>0 and later_hidden is None: later_hidden=row['t_sim_s']
        frames+=1
    passed=bool(header['map']==fixture['map'] and first is not None and visible in first
                and hidden not in first and later_hidden is not None and frames>1 and categories_valid)
    return dict(evidence_kind='synthetic_engine_visibility_supervision',passed=passed,detour_frames=frames,
                initially_visible_chair=bool(first and visible in first),
                initially_hidden_lamp=bool(first is not None and hidden not in first),
                known_categories_match=categories_valid,
                first_lamp_detection_sim_s=later_hidden,
                interpretation='bbox-sample collision-ray supervision; no rendered image or visual recognition tested')
