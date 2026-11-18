import json
from pathlib import Path
import tempfile
import unittest
from ai_worker_sorting.simulation_result import begin_result,fail_result


class SimulationResultTests(unittest.TestCase):
    def test_startup_failure_cannot_leave_previous_success(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'simulation_result.json'
            path.write_text('{"success":true,"run_id":"old"}')
            current = begin_result(folder)
            fail_result(folder,current,ValueError('weights missing'))
            report = json.loads(path.read_text())
            self.assertFalse(report['success'])
            self.assertEqual(report['run_id'],current)
            self.assertEqual(report['error'],'weights missing')

    def test_failure_preserves_current_run_measurements(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'simulation_result.json'
            current = begin_result(folder)
            path.write_text(json.dumps(dict(run_id=current,objects={'a':{'on_table':False}})))
            fail_result(folder,current,RuntimeError('placement failed'))
            self.assertFalse(json.loads(path.read_text())['objects']['a']['on_table'])
