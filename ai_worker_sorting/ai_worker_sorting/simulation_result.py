"""Replace previous run results before starting, including startup failure paths."""
import json
from pathlib import Path
import uuid


def begin_result(folder):
    run_id = uuid.uuid4().hex
    (Path(folder)/'simulation_result.json').write_text(json.dumps(
        dict(run_id=run_id,success=False,state='STARTING',message='Simulation is starting'),indent=2))
    return run_id


def fail_result(folder,run_id,error):
    path = Path(folder)/'simulation_result.json'
    try:
        report = json.loads(path.read_text())
    except (OSError,ValueError):
        report = {}
    if report.get('run_id')!=run_id:
        report = {}
    report.update(run_id=run_id,success=False,state='FAILED',error=str(error))
    path.write_text(json.dumps(report,indent=2))
