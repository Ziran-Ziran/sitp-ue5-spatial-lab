"""Audit all three fixed marker planner variants; never select successful runs."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from continuous_grid_planner import VARIANTS
from depth_path_experiment import completed_job
from navigation_build import verify_build,file_hash
from navigation_experiment import read
from online_depth_experiment import policy_sources
from visual_marker_experiment import control_acceptance
from visual_marker_report import evaluate
from visual_marker_tasks import validate_tasks


def unique_argument(command,expected):
    key=expected.split('=',1)[0].lower()
    if [t for t in command if t.split('=',1)[0].lower()==key]!=[expected]:
        raise ValueError('unique exact marker producer argument required: '+key)


def audit_run(directory,project,build):
    directory=Path(directory).absolute();project=Path(project).absolute()
    tasks=validate_tasks(read(directory/'tasks.json'))
    if tasks['schema_version']!=2:raise ValueError('explicit version-two planner condition required')
    game=completed_job(directory/'game.job.json');fixture=completed_job(directory/'fixture.job.json')
    game_hashes={Path(p).absolute():sha for p,sha in game['input_sha256'].items()}
    fixture_hashes={Path(p).absolute():sha for p,sha in fixture['input_sha256'].items()}
    binary=(ROOT/build['binary']).absolute();script=ROOT/'ue5/tools/visual_marker_fixture.py'
    for producer,hashes in ((game,game_hashes),(fixture,fixture_hashes)):
        command=producer['command']
        if len(command)<3 or Path(command[1]).absolute()!=project or sum(t.lower().endswith('.uproject') for t in command)!=1:
            raise ValueError('wrong marker project')
        if hashes.get(binary)!=build['binary_sha256']:raise ValueError('marker variants must use the current same native DLL')
    if game['command'][2]!=tasks['map']:raise ValueError('wrong actual marker map')
    for argument in ('-game','-SITPVisualMarker='+str(directory/'tasks.json').replace('\\','/'),
            '-SITPResult='+str(directory/'execution.json').replace('\\','/'),'-SITPOnlineOutput='+str(directory/'capture').replace('\\','/'),
            '-SITPTools='+str(ROOT/'ue5/tools').replace('\\','/'),'-SITPPython='+str(Path(sys.executable)).replace('\\','/')):
        unique_argument(game['command'],argument)
    argv=[str(script).replace('\\','/'),'--map',tasks['map'],'--out',str(directory/'fixture.json').replace('\\','/'),
        '--truth',str(directory/'truth.json').replace('\\','/')]
    unique_argument(fixture['command'],'-run=pythonscript');unique_argument(fixture['command'],'-script='+subprocess.list2cmdline(argv))
    if fixture_hashes.get(script)!=file_hash(script):raise ValueError('changed marker fixture generator')
    required={directory/'tasks.json':file_hash(directory/'tasks.json'),directory/'truth.json':file_hash(directory/'truth.json'),
        project.parent/'Content'/(tasks['map'][len('/Game/'):]+'.umap'):tasks['map_sha256']}
    required.update({p:file_hash(p) for p in policy_sources('visual_marker_depth_policy')})
    required[ROOT/'scripts/visual_marker_experiment.py']=file_hash(ROOT/'scripts/visual_marker_experiment.py')
    if any(game_hashes.get(p)!=sha or file_hash(p)!=sha for p,sha in required.items()):raise ValueError('marker inputs/source differ from actual locked producer')
    fixture_record=read(directory/'fixture.json')
    if fixture_record['map']!=tasks['map'] or fixture_record['navmesh_build_called'] is not False or fixture_record['navigation_data_actors']!=0:
        raise ValueError('wrong marker fixture evidence')
    record=read(directory/'execution.json');truth=read(directory/'truth.json');report=evaluate(tasks,record,directory/'capture',truth)
    acceptance=control_acceptance(report,record)
    if not acceptance['passed']:raise ValueError('incomplete or invalid marker experiment')
    return dict(variant=tasks['planner_variant'],report=report,truth_entities=truth['entities'],
        fixture_source_sha256=file_hash(script),native_binary_sha256=build['binary_sha256'],
        raw_execution_sha256=file_hash(directory/'execution.json'),manifest_sha256=tasks['manifest_sha256'],
        game_spec_sha256=file_hash(directory/'game.job.json'),game_status_sha256=file_hash(directory/'game.job.status.json'))


def compare(audited):
    if len(audited)!=3 or {r['variant'] for r in audited}!=set(VARIANTS):raise ValueError('all three unique predeclared variants required')
    for key in ('truth_entities','fixture_source_sha256','native_binary_sha256'):
        if any(row[key]!=audited[0][key] for row in audited):raise ValueError('variants change controlled world or native build')
    return dict(schema_version=1,evidence_kind='actual_fixed_visual_marker_planner_comparison',
        variants=[dict(variant=variant,**next(r for r in audited if r['variant']==variant)['report']['overall']) for variant in VARIANTS],
        provenance=[{k:v for k,v in row.items() if k not in ('report','truth_entities')} for row in audited],
        interpretation='fixed synthetic color markers, all twelve tasks per variant; same generator/native DLL, extra observations and costs retained; no city/furniture/generalization claim')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--runs',nargs=3,type=Path,required=True)
    parser.add_argument('--project',type=Path,required=True);parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    build=verify_build(ROOT,args.project.absolute());result=compare([audit_run(d,args.project,build) for d in args.runs])
    with args.out.open('x',encoding='utf-8') as target:json.dump(result,target,indent=2,allow_nan=False)
    print(json.dumps(result['variants'],indent=2))


if __name__=='__main__':main()
