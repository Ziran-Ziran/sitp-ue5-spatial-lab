"""Reconstruct and audit both complete online numerical-index conditions."""
import argparse
import copy
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from compare_visual_marker_runs import unique_argument
from depth_path_experiment import completed_job
from navigation_build import verify_build,file_hash
from navigation_experiment import read
from online_depth_experiment import prepare,verify_controls,policy_sources
from online_depth_report import evaluate
from online_depth_tasks import validate_tasks,digest
from query_grid_index import POLICIES


def audit_run(directory,controls,project,build,capture,capture_job):
    directory=Path(directory).absolute();controls=Path(controls).absolute();project=Path(project).absolute()
    tasks=validate_tasks(read(directory/'tasks.json'));policy=tasks.get('query_cell_policy')
    if tasks['schema_version']!=2 or policy not in POLICIES:raise ValueError('explicit version-two query policy required')
    if tasks!=prepare(capture,capture_job,project,query_cell_policy=policy):raise ValueError('changed original full source tasks')
    acceptance=verify_controls(controls,project,build)
    if not acceptance['passed'] or read(controls/'tasks.json').get('query_cell_policy')!=policy:
        raise ValueError('same-policy current controls required')
    game=completed_job(directory/'game.job.json');command=game['command']
    if len(command)<3 or Path(command[1]).absolute()!=project or sum(t.lower().endswith('.uproject') for t in command)!=1 or command[2]!=tasks['map']:
        raise ValueError('wrong online query project/map')
    for arg in ('-game','-SITPOnlineDepth='+str(directory/'tasks.json').replace('\\','/'),
            '-SITPResult='+str(directory/'execution.json').replace('\\','/'),'-SITPOnlineOutput='+str(directory/'capture').replace('\\','/'),
            '-SITPTools='+str(ROOT/'ue5/tools').replace('\\','/'),'-SITPPython='+str(Path(sys.executable)).replace('\\','/')):
        unique_argument(command,arg)
    hashes={Path(p).absolute():sha for p,sha in game['input_sha256'].items()}
    required={ROOT/build['binary']:build['binary_sha256'],directory/'tasks.json':file_hash(directory/'tasks.json'),
        project.parent/'Content'/(tasks['map'][len('/Game/'):]+'.umap'):tasks['map_sha256']}
    required.update({p:file_hash(p) for p in policy_sources()})
    launcher=ROOT/'scripts/online_depth_experiment.py';required[launcher]=file_hash(launcher)
    for name in ('tasks.json','game.job.json','game.job.status.json','fixture.job.json','fixture.job.status.json'):
        path=controls/name;required[path]=file_hash(path)
    if any(hashes.get(p)!=sha or file_hash(p)!=sha for p,sha in required.items()):raise ValueError('query run changes executed input/source/control bindings')
    raw_hash=file_hash(directory/'execution.json');record=read(directory/'execution.json');report=evaluate(tasks,record,directory/'capture')
    if file_hash(directory/'execution.json')!=raw_hash:raise ValueError('raw execution changed during audit')
    if not report['finished'] or any(r['failure_reason']=='invalid_execution' for r in report['runs']):raise ValueError('incomplete/invalid query condition')
    canonical=copy.deepcopy(tasks);canonical.pop('query_cell_policy');canonical['schema_version']=1
    for run in canonical['runs']:run.pop('query_cell_policy')
    return dict(policy=policy,native_binary_sha256=build['binary_sha256'],canonical_task_sha256=digest(canonical),report=report,
        manifest_sha256=tasks['manifest_sha256'],raw_execution_sha256=raw_hash,
        game_spec_sha256=file_hash(directory/'game.job.json'),game_status_sha256=file_hash(directory/'game.job.status.json'))


def compare(audited):
    if len(audited)!=2 or {r['policy'] for r in audited}!=set(POLICIES):raise ValueError('both unique predeclared query policies required')
    for key in ('native_binary_sha256','canonical_task_sha256'):
        if audited[0][key]!=audited[1][key]:raise ValueError('query conditions change native build or fixed tasks')
    return dict(schema_version=1,evidence_kind='actual_online_query_index_policy_comparison',
        conditions=[dict(policy=p,overall=next(r for r in audited if r['policy']==p)['report']['overall'],
            by_mode=next(r for r in audited if r['policy']==p)['report']['by_mode']) for p in POLICIES],
        provenance=[{k:v for k,v in r.items() if k!='report'} for r in audited],
        interpretation='all original source poses/modes and raw continuous query coordinates retained; explicit engine pose supervision; numerical indexing comparison, not semantic search or generalization')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',nargs=2,type=Path,required=True);parser.add_argument('--controls',nargs=2,type=Path,required=True)
    for name in ('project','capture','capture-job','out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();build=verify_build(ROOT,args.project.absolute())
    result=compare([audit_run(d,c,args.project,build,args.capture.absolute(),args.capture_job.absolute()) for d,c in zip(args.runs,args.controls)])
    with args.out.open('x',encoding='utf-8') as target:json.dump(result,target,indent=2,allow_nan=False)
    print(json.dumps(result['conditions'],indent=2))


if __name__=='__main__':main()
