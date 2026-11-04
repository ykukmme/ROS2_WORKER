"""Ubuntu terminal operator menu. Launching the menu never starts motion."""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time
import uuid
import yaml

TYPE_LABELS = ('첫 번째 종류','두 번째 종류')


class RosBackend:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from ai_worker_interfaces.srv import PrepareAutoSort,StartTask,TaskStatus,CancelTask,SessionStatus
        self.rclpy = rclpy
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            # The operator menu handles Ctrl+C without losing status/cancel RPCs.
            rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node('worker_terminal_menu')
        self.types = dict(prepare_auto_sort=PrepareAutoSort,start_task=StartTask,
            task_status=TaskStatus,cancel_task=CancelTask,session_status=SessionStatus)
        self.clients = {k:self.node.create_client(v,'/worker/'+k) for k,v in self.types.items()}
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin,daemon=True)
        self.thread.start()

    def visible(self):
        return self.clients['start_task'].wait_for_service(timeout_sec=.2)

    def begin(self,name,**values):
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('작업 서버에 연결할 수 없습니다.')
        return client.call_async(self.types[name].Request(**values))

    def result(self,future,timeout=15.):
        from rosidl_runtime_py.convert import message_to_ordereddict
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.05)
        if not future.done():
            raise TimeoutError('응답 대기 시간 초과. 시작 명령을 다시 보내지 마세요.')
        if future.result() is None:
            raise RuntimeError('작업 서버 응답이 없습니다.')
        return dict(message_to_ordereddict(future.result()))

    def request(self,name,**values):
        return self.result(self.begin(name,**values))

    def close(self):
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(timeout=5)
        self.node.destroy_node()
        if self.owns_context:
            self.rclpy.try_shutdown()


class ManagedStack:
    def __init__(self,directory):
        self.directory = directory
        self.process,self.stream,self.mode = None,None,None

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def start(self,mode,command):
        if self.alive():
            raise RuntimeError('이미 실행 중인 내부 노드가 있습니다.')
        self.stop()
        self.directory.mkdir(parents=True,exist_ok=True)
        self.stream = (self.directory/(mode+'.log')).open('a',encoding='utf-8')
        self.process = subprocess.Popen(command,stdout=self.stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.mode = mode

    def stop(self):
        # Only menu-owned processes; call only when no task is active/unknown.
        if self.alive():
            for sig,timeout in ((signal.SIGINT,10),(signal.SIGTERM,5),(signal.SIGKILL,3)):
                if not self.alive():
                    break
                os.killpg(self.process.pid,sig)
                try:
                    self.process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    pass
        if self.stream:
            self.stream.close()
        self.process,self.stream,self.mode = None,None,None


class TerminalMenu:
    def __init__(self,config,backend,ask=input,show=print,stack=None):
        self.config,self.backend,self.ask,self.show = Path(config),backend,ask,show
        self.stack = stack or ManagedStack(self.config.parent/'menu-logs')
        self.session,self.task,self.pending = None,None,None
        self.start_unconfirmed = False
        self.state = 'IDLE'
        self.rules = []

    def resolve_pending(self):
        if self.pending is not None and self.pending.done():
            response = self.backend.result(self.pending)
            self.pending = None
            self.start_unconfirmed = False
            if response['accepted']:
                self.task,self.state = response['task_id'],'QUEUED'
            else:
                self.state = 'IDLE'
                raise RuntimeError(response.get('message','작업이 거부됐습니다.'))

    def status(self):
        self.resolve_pending()
        if self.pending is not None or self.start_unconfirmed:
            self.show('시작 응답 확인 중: 재시작·설정 변경·노드 종료를 차단합니다.')
            return False
        if not self.task:
            self.show('진행 중인 작업이 없습니다.')
            return True
        result = self.backend.request('task_status',task_id=self.task)
        if not result['found']:
            raise RuntimeError('작업 상태를 확인할 수 없습니다. 장비 상태를 확인하세요.')
        self.state = result['state']
        self.show(f"작업: {self.state} / 단계: {result['stage']}")
        if self.session:
            session = self.backend.request('session_status',session_id=self.session)
            if session['found']:
                self.show(f"배치 완료: {session['placed_count']}개")
        return self.state in ('SUCCEEDED','FAILED','CANCELED')

    def idle(self):
        return self.status() if self.task or self.pending is not None or self.start_unconfirmed else True

    def command(self,module,*args,timeout=30):
        result = subprocess.run([sys.executable,'-m','ai_worker_sorting.'+module,*map(str,args)],
            capture_output=True,text=True,timeout=timeout)
        logs = self.config.parent/'menu-logs'
        logs.mkdir(parents=True,exist_ok=True)
        with (logs/'commands.log').open('a',encoding='utf-8') as stream:
            stream.write(f'\n{module}\n{result.stdout}\n{result.stderr}\n')
        if result.returncode:
            detail = result.stderr.strip() or result.stdout.strip() or '실행 실패'
            raise RuntimeError(detail.splitlines()[-1]+' (상세: menu-logs/commands.log)')
        return result.stdout

    def launch(self,mode,launch,*args):
        if self.backend.visible():
            raise RuntimeError('별도로 실행 중인 분류 서버를 종료한 후 메뉴를 사용하세요.')
        self.stack.start(mode,['ros2','launch','ai_worker_sorting',launch,
            'python_prefix:='+sys.executable,*map(str,args)])

    def ensure_live(self):
        if self.stack.alive() and self.stack.mode=='live':
            return
        if not self.config.is_file():
            raise RuntimeError('먼저 초기 설정을 완료하세요.')
        self.command('worker_cli','doctor','--config',self.config)
        self.launch('live','ai_worker_live.launch.py','config:='+str(self.config))
        deadline = time.monotonic()+30
        while time.monotonic()<deadline:
            if not self.stack.alive():
                break
            if self.backend.visible():
                return
        self.stack.stop()
        raise RuntimeError('분류 서버 시작 실패. menu-logs/live.log를 확인하세요.')

    def prepare(self):
        if not self.idle():
            raise RuntimeError('현재 작업이 끝난 뒤 다시 확인하세요.')
        self.session,self.rules = None,[]
        self.ensure_live()
        result = self.backend.request('prepare_auto_sort')
        if not result['success']:
            raise RuntimeError(result['message'])
        self.session,self.rules = result['session_id'],result['rules']
        self.show('인식한 두 종류와 목적 구역을 확인했습니다.')
        for label,rule in zip(TYPE_LABELS,self.rules):
            self.show(f"{label} → {rule['zone_id']}")

    def start(self):
        if not self.idle():
            raise RuntimeError('현재 작업의 종료 상태부터 확인하세요.')
        # Refresh the observation/zone assignment immediately before confirmation.
        self.prepare()
        self.show('팔을 작업물에서 떨어뜨린 뒤 시작하세요.')
        self.show('카메라를 가리지 않고 위로 이동할 공간이 있어야 하며, 그리퍼는 비어 있어야 합니다.')
        count = self.ask('처리 개수 [기본 2, 모두 처리 0]: ').strip() or '2'
        if not count.isdigit() or not 0<=int(count)<=4294967295:
            raise ValueError('0 이상의 개수를 입력하세요.')
        if self.ask('실제 분류를 시작할까요? [y/N]: ').strip().lower()!='y':
            return False
        # Keep the future if the response times out; never resend automatically.
        self.state = 'START_PENDING'
        self.start_unconfirmed = True
        self.pending = self.backend.begin('start_task',session_id=self.session,max_objects=int(count))
        self.backend.result(self.pending)
        self.resolve_pending()
        return True

    def cancel(self):
        self.resolve_pending()
        if self.pending is not None or self.start_unconfirmed:
            raise RuntimeError('시작 응답을 기다리는 중입니다. 실제 장비 상태를 확인하세요.')
        if not self.task:
            self.show('취소할 작업이 없습니다.')
            return
        response = self.backend.request('cancel_task',task_id=self.task)
        self.show('취소 요청을 보냈습니다. 종료 상태와 실제 장비를 확인하세요.' if response['accepted'] else '취소가 접수되지 않았습니다. 상태를 확인하세요.')
        self.status()

    def wizard(self):
        if not self.idle():
            raise RuntimeError('작업 종료를 먼저 확인하세요.')
        if not self.allow_stop():
            return
        if self.config.exists() and self.ask('저장된 설정을 새로 지정할까요? [y/N]: ').strip().lower()!='y':
            return
        self.stack.stop()
        self.session,self.task,self.rules = None,None,[]
        old = yaml.safe_load(self.config.read_text()) if self.config.exists() else {}
        worker = old.get('worker_service',{}).get('ros__parameters',{})
        camera = old.get('rgbd_input',{}).get('ros__parameters',{})
        detector = old.get('rgbd_detector',{}).get('ros__parameters',{})
        def field(text,default):
            return self.ask(f'{text} [{default}]: ').strip() or str(default)
        weights = Path(field('로컬 YOLO 가중치 경로',detector.get('weights',''))).expanduser().resolve()
        if not weights.is_file():
            raise ValueError('가중치 파일이 없습니다.')
        workspace = field('작업 범위 xmin,xmax,ymin,ymax,zmin,zmax',','.join(map(str,worker.get('workspace',[]))))
        values = [float(x) for x in workspace.split(',')]
        import math
        if len(values)!=6 or not all(math.isfinite(x) for x in values) or any(values[i]>=values[i+1] for i in (0,2,4)):
            raise ValueError('작업 범위의 숫자 6개를 확인하세요.')
        device = field('추론 장치',detector.get('device','cpu'))
        base = worker.get('base_frame','base_link')
        topics = dict(color_topic=camera.get('color_topic','/camera/color/image_raw'),
            depth_topic=camera.get('depth_topic','/camera/aligned_depth_to_color/image_raw'),
            camera_info_topic=camera.get('camera_info_topic','/camera/color/camera_info'))
        prefix = self.config.parent/('setup-'+uuid.uuid4().hex)
        state,output = Path(str(prefix)+'.local.yaml'),Path(str(prefix)+'-live.local.yaml')
        try:
            self.launch('calibration','calibration.launch.py','weights:='+str(weights),'device:='+device,
                'base_frame:='+base,*[k+':='+v for k,v in topics.items()])
            self.ask('두 종류의 물건 전체와 빈 테이프 구역을 보이게 놓고 Enter를 누르세요. ')
            options = ['--weights',str(weights),'--workspace='+workspace,'--device',device,
                '--base-frame',base,'--controller-frame',worker.get('controller_frame','base_link'),
                '--controller-node',worker.get('controller_node','/ai_worker_movel_controller'),
                '--movel-topic',worker.get('movel_topic','/l_goal_move'),
                '--gripper-topic',worker.get('gripper_trajectory_topic','/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory'),
                '--joint-states-topic',worker.get('joint_states_topic','/joint_states'),
                '--gripper-joint',worker.get('gripper_joint','gripper_l_joint1')]
            for key,value in topics.items():
                options += ['--'+key.replace('_','-'),value]
            if worker.get('source_roi'):
                options += ['--source-roi',','.join(map(str,worker['source_roi']))]
            if os.environ.get('WORKER_ROBOT_DESCRIPTION_NODE'):
                options += ['--robot-description-node',os.environ['WORKER_ROBOT_DESCRIPTION_NODE']]
            if os.environ.get('WORKER_ROBOT_URDF'):
                options += ['--robot-urdf',os.environ['WORKER_ROBOT_URDF']]
            deadline = time.monotonic()+45
            while True:
                try:
                    self.command('configure_auto','scan','--state',state,*options)
                    break
                except Exception:
                    if state.exists() or time.monotonic()>deadline or not self.stack.alive():
                        raise
                    self.show('카메라·인식 결과를 기다리는 중입니다…')
                    time.sleep(1)
            data = yaml.safe_load(state.read_text())
            self.show('물건·카메라를 움직이지 마세요. 그리퍼 형상과 장착/TCP가 정확한 URDF·TF를 사용합니다.')
            self.show('첫 집기 자세에서 모델 형상을 읽어 그리퍼 길이를 계산합니다. 빈 테이블 기준 지정은 생략합니다.')
            for label,name in zip(TYPE_LABELS,data['samples']):
                # Labels are discovered, never restricted to predefined products.
                self.ask(f'{label}(현재 인식명: {name})를 집을 자세를 지정한 뒤 Enter: ')
                self.command('configure_auto','grasp','--state',state,'--class-name',name,'--grasp-pose-confirmed')
            self.ask('팔을 작업물에서 떨어뜨리세요. 집기 방향을 유지한 뒤 Enter: ')
            self.command('configure_auto','write','--state',state,'--output',output)
            os.replace(output,self.config)
            self.show('설정을 저장했습니다. 다음부터 재사용합니다.')
        finally:
            self.stack.stop()

    def watch(self):
        self.show('상태 자동 갱신: c + Enter 취소 / Enter 메뉴로 돌아가기')
        while True:
            try:
                finished = self.status()
            except Exception as exc:
                self.show('상태 확인 실패: '+str(exc))
                finished = False
            if finished:
                return
            if select.select([sys.stdin],[],[],1.)[0]:
                value = sys.stdin.readline()
                if not value:
                    raise EOFError()
                if value.strip().lower()=='c':
                    self.cancel()
                else:
                    return

    def allow_stop(self):
        if self.state=='FAILED' and self.stack.alive():
            self.show('작업 실패는 실제 정지 확인이 아닙니다. 제어기·장비 상태를 확인하세요.')
            return self.ask('장비 정지를 직접 확인했고 내부 노드를 종료할까요? [y/N]: ').strip().lower()=='y'
        return True

    def exit(self):
        try:
            if not self.idle():
                self.show('작업 중이거나 상태 미확인입니다. 취소 후 종료 상태를 확인하세요.')
                return False
            if not self.allow_stop():
                return False
        except Exception as exc:
            self.show(str(exc))
            return False
        self.stack.stop()
        return True

    def run(self):
        while True:
            try:
                self.show('\nAI WORKER 분류 — 설정: '+('있음' if self.config.is_file() else '없음'))
                self.show('1 초기 설정 / 2 물건·구역 확인 / 3 분류 시작 / 4 작업 상태 / 5 작업 취소 / 6 연결 검사 / 0 종료')
                option = self.ask('선택: ').strip()
                if option=='0':
                    if self.exit():
                        return 0
                elif option=='1':
                    self.wizard()
                elif option=='2':
                    self.prepare()
                elif option=='3':
                    if self.start():
                        self.watch()
                elif option=='4':
                    self.watch()
                elif option=='5':
                    self.cancel()
                elif option=='6':
                    self.command('worker_cli','doctor','--config',self.config)
                    self.show('연결 검사 통과. 움직임이나 집기 성공을 확인한 것은 아닙니다.')
                else:
                    self.show('메뉴 번호를 선택하세요.')
            except KeyboardInterrupt:
                self.show('종료 요청: 실행 중인 작업은 먼저 취소하세요.')
            except EOFError:
                # Never kill a possibly moving controller because stdin disappeared.
                if not self.exit():
                    self.show('입력 연결이 끊겼습니다. 작업 서버를 유지합니다. 장비와 로그를 확인하세요.')
                return 2
            except Exception as exc:
                self.show('확인 필요: '+str(exc))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    args = parser.parse_args()
    if sys.platform!='linux' or not sys.stdin.isatty():
        parser.error('Ubuntu의 대화형 터미널에서 실행하세요.')
    backend = RosBackend()
    try:
        return TerminalMenu(args.config.expanduser().resolve(),backend).run()
    finally:
        backend.close()


if __name__=='__main__':
    sys.exit(main())
