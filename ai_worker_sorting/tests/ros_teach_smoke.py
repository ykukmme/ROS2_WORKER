"""Read-only teaching workflow with isolated emulated hardware, no physical robot."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from ai_worker_sorting.core import Point
from ai_worker_sorting.zone_service import ZoneService
from ai_worker_sorting.rgbd_input import AlignedRGBDInput
from ros_live_smoke import ControllerFixture


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    fixture = ControllerFixture()
    fixture.height_peak = .04
    for obj in fixture.objects:
        p = obj['point']
        obj['pixel'] = (320+500*(p.x-.4)/(1-p.z),240-500*p.y/(1-p.z))
    zones = ZoneService(parameter_overrides=[Parameter('mock_mode',value=True),Parameter('base_frame',value='live_test_base')])
    original = zones.mock_zones
    def observed_zones():
        result = original()
        for zone in result:
            zone.simulated = False
        return result
    zones.mock_zones = observed_zones
    rgbd = AlignedRGBDInput(parameter_overrides=[Parameter('color_topic',value='/live_test/color'),
        Parameter('depth_topic',value='/live_test/depth'),Parameter('camera_info_topic',value='/live_test/info')])
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (fixture,zones,rgbd):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    cases = []
    try:
        with tempfile.TemporaryDirectory() as directory:
            state,profile = Path(directory)/'teach.local.yaml',Path(directory)/'live.local.yaml'
            def invoke(stage,*options,expected=0):
                result = subprocess.run([sys.executable,'-m','ai_worker_sorting.configure_auto',stage,
                    '--state',str(state),*options],capture_output=True,text=True,timeout=20)
                assert result.returncode==expected,(result.stdout,result.stderr)
                return yaml.safe_load(state.read_text())
            data = invoke('scan','--weights',args.weights,'--workspace=-.1,.7,-.4,.4,.05,.8',
                '--base-frame','live_test_base','--controller-frame','live_test_base',
                '--controller-node','/live_test_controller_fixture','--movel-topic','/live_test/movel',
                '--gripper-topic','/live_test/gripper','--joint-states-topic','/live_test/joints',
                '--color-topic','/live_test/color','--depth-topic','/live_test/depth','--camera-info-topic','/live_test/info')
            assert .06<data['object_size']<.15 and .11<data['max_object_height']<.13
            cases.append('camera scan derives two class envelopes and height bound without dimension arguments')
            invoke('write','--output',str(profile),expected=2)
            assert not profile.exists()
            cases.append('incomplete teaching cannot generate a live configuration')
            fixture.pose = Point(.35,0.,.12)
            fixture.quaternion = [-1.,0.,0.,0.]
            data = invoke('tool','--tips-at-table')
            assert abs(data['tool_below_tcp']-.023)<1e-6
            for name,z in [('can',.13),('cup',.14)]:
                sample = data['samples'][name]
                fixture.pose = Point(sample[0],sample[1],z)
                data = invoke('grasp','--class-name',name,'--grasp-pose-confirmed')
            assert abs(data['class_offsets']['can']+.02)<1e-6
            assert abs(data['class_offsets']['cup']+.01)<1e-6
            cases.append('fresh tool/table reference and two taught poses produce distinct grasp offsets')
            invoke('write','--output',str(profile))
            params = yaml.safe_load(profile.read_text())['worker_service']['ros__parameters']
            assert params['check_camera_footprint'] is True
            assert set(json.loads(params['class_grasp_offsets']))=={'can','cup'}
            cases.append('teaching writes a real, camera-checked live profile using the existing hardware validator')
            assert not fixture.moves and not fixture.grips
            cases.append('entire workflow sends zero MoveL or gripper commands')
        report = dict(result='PASS',mode='emulated_hardware_readonly_teaching',cases=cases)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for node in (rgbd,zones,fixture):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
