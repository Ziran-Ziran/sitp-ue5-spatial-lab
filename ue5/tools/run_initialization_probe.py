"""One declared initialization-angle probe, isolated DB/output, no truth inputs."""
import json
from pathlib import Path
import shutil
import sys
import time

import pycolmap

from diagnose_reconstruction import digest


def main():
    root=Path(__file__).resolve().parents[2]
    base=root/'output/ue5-eval/diagnosis-20261026-0946'
    design=json.loads((base/'initialization-design.json').read_text())
    db=base/'database-angle1-probe.db'
    out=root/'output/ue5-eval/map-night-city-lap2-a-init-bridge-angle1'
    assert not db.exists() and not out.exists()
    shutil.copy2(base/'database-before-init-header-recovered.db',db)
    assert digest(db)==design['input_db_sha256']
    out.mkdir()
    options=pycolmap.IncrementalPipelineOptions()
    options.num_threads=8;options.random_seed=0;options.mapper.random_seed=0
    options.init_image_id1,options.init_image_id2=design['image_ids']
    options.mapper.init_min_tri_angle=1.0
    settings=dict(changed_from_failed_forced_pair_control='mapper.init_min_tri_angle: 16 to 1 degrees only',
                  initial_frames=design['frames'],initial_ids=design['image_ids'],
                  ordinary_triangulation_min_angle=options.triangulation.min_angle,
                  options=options.todict(),ground_truth_read=False,input_database=str(db),
                  input_database_original_content_sha256=design['input_db_sha256'])
    (out/'probe-settings.json').write_text(json.dumps(settings,default=str,indent=2),encoding='utf-8')
    started=time.monotonic()
    models=pycolmap.incremental_mapping(db,root/'output/ue5-datasets/night-city-lap2-a/frames',out/'sparse',options=options)
    if not models:
        print('no reconstruction produced',flush=True);return 2
    best=max(models.values(),key=lambda r:r.num_reg_images())
    (out/'best').mkdir();best.write(out/'best')
    summary=dict(session='night-city-lap2-a',arm='init bridge 123-139 angle1 seed0',
                 registered=best.num_reg_images(),frames=492,registered_fraction=best.num_reg_images()/492,
                 points3d=best.num_points3D(),models=len(models),mapping_s=time.monotonic()-started,
                 ground_truth_read=False,output_cameras={str(k):str(v) for k,v in best.cameras.items()})
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8');print(json.dumps(summary),flush=True)
    return 0


if __name__=='__main__':sys.exit(main())
