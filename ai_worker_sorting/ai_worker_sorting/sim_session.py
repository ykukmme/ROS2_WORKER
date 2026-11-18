"""Start isolated simulation nodes and expose existing ROS2 services in a terminal menu."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ai_worker_interfaces.srv import PrepareAutoSort, StartTask, TaskStatus, CancelTask

from .auto_worker import AutoWorker
from .rgbd_input import AlignedRGBDInput
from .rgbd_detector import RGBDDetector
from .vision_service import VisionService
from .zone_service import ZoneService
from .sim_bridge import SimulationBridge
from .sim_profile import profile, validate_profile, resolve_weights
from .sim_scene import OBJECTS, TABLE_Z, OBJECT_HEIGHT
from .simulation_result import begin_result,fail_result


class Session:
    def __init__(self, output, run_id=''):
        self.output = output
        self.run_id = run_id
        self.config = validate_profile(profile(output/'simulation_yolo.pt'))
        self.nodes, self.task_id, self.prepared = [], '', None
        self.executor = MultiThreadedExecutor(num_threads=10)
        self.executor_thread = None
        try:
            self.bridge = SimulationBridge()
            self.nodes.append(self.bridge)
            constructors = dict(rgbd_input=AlignedRGBDInput,rgbd_detector=RGBDDetector,
                                vision_service=VisionService,zone_service=ZoneService,worker_service=AutoWorker)
            for name, constructor in constructors.items():
                params = [Parameter(k,value=v) for k,v in self.config[name].items()]
                self.nodes.append(constructor(parameter_overrides=params))
            self.client = Node('simulation_terminal',parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.nodes.append(self.client)
            self.clients = {
                'prepare': self.client.create_client(PrepareAutoSort,'/worker/prepare_auto_sort'),
                'start': self.client.create_client(StartTask,'/worker/start_task'),
                'status': self.client.create_client(TaskStatus,'/worker/task_status'),
                'cancel': self.client.create_client(CancelTask,'/worker/cancel_task'),
            }
            for node in self.nodes:
                self.executor.add_node(node)
            self.executor_thread = threading.Thread(target=self.executor.spin,daemon=True)
            self.executor_thread.start()
        except Exception:
            self.close()
            raise

    def call(self, name, request, timeout=30):
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=3):
            raise TimeoutError(f'Service unavailable: {name}')
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _:event.set())
        if not event.wait(timeout):
            client.remove_pending_request(future)
            raise TimeoutError(f'Service timed out: {name}')
        return future.result()

    def prepare(self, wait=40):
        deadline = time.monotonic()+wait
        result = None
        while time.monotonic()<deadline:
            with self.bridge.lock:
                if self.bridge.world.fault:
                    raise RuntimeError(self.bridge.world.fault)
            result = self.call('prepare',PrepareAutoSort.Request())
            if result.success:
                self.prepared = result
                print('ZONE_ASSIGNMENT',[(r.object_class,r.zone_id) for r in result.rules],
                      [(z.zone_id,z.color) for z in result.zones],flush=True)
                return result
            time.sleep(.4)
        raise RuntimeError('Preparation failed: '+(result.message if result else 'no response'))

    def start(self):
        if self.task_id:
            raise RuntimeError('이 장면은 이미 시작했습니다. 종료 후 다시 실행하세요.')
        if self.prepared is None:
            self.prepare()
        result = self.call('start',StartTask.Request(session_id=self.prepared.session_id,max_objects=0))
        if not result.accepted:
            raise RuntimeError(result.message)
        self.task_id = result.task_id
        return result

    def status(self):
        return self.call('status',TaskStatus.Request(task_id=self.task_id))

    def wait(self, timeout=900):
        last = None
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            result = self.status()
            state = result.state,result.stage,result.message
            if state!=last:
                print('TASK',*state,flush=True)
                last = state
            with self.bridge.lock:
                fault = self.bridge.world.fault
            if fault:
                self.cancel()
                raise RuntimeError(fault)
            if result.state in ('SUCCEEDED','FAILED','CANCELED'):
                return result
            time.sleep(.3)
        self.cancel()
        raise TimeoutError('Simulation task exceeded wall-clock watchdog')

    def cancel(self):
        if self.task_id:
            return self.call('cancel',CancelTask.Request(task_id=self.task_id))

    def evaluate(self, task_result):
        import mujoco
        from PIL import Image
        from .zones import inside
        with self.bridge.lock:
            actual = self.bridge.world.evaluation()
            fault = self.bridge.world.fault
            history = set(self.bridge.lifted_objects)
            snapshot = mujoco.MjData(self.bridge.world.model)
            snapshot.qpos[:] = self.bridge.world.data.qpos
        mujoco.mj_forward(self.bridge.world.model,snapshot)
        with mujoco.Renderer(self.bridge.world.model,height=480,width=640) as renderer:
            renderer.update_scene(snapshot,camera='work_camera')
            Image.fromarray(renderer.render().copy()).save(self.output/'final_scene.png')
        mapping = {r.object_class:r.zone_id for r in self.prepared.rules}
        zones = {z.zone_id:[(p.x,p.y) for p in z.boundary.polygon.points] for z in self.prepared.zones}
        checks = {}
        for name, label, _ in OBJECTS:
            p = actual[name]['position']
            checks[name] = dict(in_assigned_zone=inside(p[:2],zones[mapping[label]]),
                                on_table=abs(p[2]-(TABLE_Z+OBJECT_HEIGHT/2))<.015,
                                contact_lift_observed=name in history,position=p)
        success = (task_result.state=='SUCCEEDED' and not fault and
                   all(c['in_assigned_zone'] and c['on_table'] and c['contact_lift_observed'] for c in checks.values()))
        report = dict(run_id=self.run_id,success=success,state=task_result.state,message=task_result.message,fault=fault,objects=checks,
                      detection_source='YOLO on rendered RGB',motion_source='MuJoCo contact dynamics')
        (self.output/'simulation_result.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        return success

    def close(self):
        worker = next((n for n in self.nodes if isinstance(n,AutoWorker)),None)
        if worker is not None:
            worker.shutdown_event.set()
            if worker.robot:
                worker.robot.cancel()
            if worker.thread:
                worker.thread.join(timeout=5)
        if self.executor_thread:
            self.executor.shutdown(timeout_sec=5)
            self.executor_thread.join(timeout=5)
        for node in reversed(self.nodes):
            node.destroy_node()
        self.nodes = []


def run(output, interactive=True):
    run_id = begin_result(output)
    initialized = False
    session = None
    preview = None
    try:
        resolve_weights(output)
        rclpy.init()
        initialized = True
        session = Session(output,run_id)
        if interactive and os.environ.get('DISPLAY'):
            import sys
            preview = subprocess.Popen([sys.executable,'-m','ai_worker_sorting.sim_preview'])
        if not interactive:
            session.prepare()
            session.start()
            result = session.wait()
            if not session.evaluate(result):
                raise RuntimeError('Full simulation verification failed')
            return
        print('물리 시뮬레이션 / 1 인식·구역 확인 / 2 분류 시작 / 3 상태 / 4 중지 / 0 종료',flush=True)
        while True:
            choice = input('선택: ').strip()
            try:
                if choice=='0':
                    break
                if choice=='1':
                    session.prepare()
                elif choice=='2':
                    session.start()
                elif choice=='3':
                    state = session.status()
                    print(state.state,state.stage,state.message,flush=True)
                    if state.state in ('SUCCEEDED','FAILED','CANCELED') and session.prepared:
                        session.evaluate(state)
                elif choice=='4':
                    session.cancel()
            except Exception as exc:
                print(str(exc),flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        fail_result(output,run_id,exc)
        raise
    finally:
        if preview is not None:
            preview.terminate()
            try:
                preview.wait(timeout=3)
            except subprocess.TimeoutExpired:
                preview.kill()
                preview.wait()
        try:
            if session is not None:
                session.close()
        except Exception as exc:
            fail_result(output,run_id,exc)
            raise
        finally:
            if initialized:
                rclpy.try_shutdown()
