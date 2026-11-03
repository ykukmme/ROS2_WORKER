"""Automatic two-class sorting through real ROS2 services, DRY robot only."""
import argparse
import json
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from ai_worker_interfaces.srv import PrepareAutoSort, StartTask, SessionStatus, TaskStatus, CancelTask
from ai_worker_sorting.vision_service import VisionService
from ai_worker_sorting.zone_service import ZoneService
from ai_worker_sorting.auto_worker import AutoWorker


def until(predicate, message, timeout=8):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.02)
    raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    vision = VisionService(parameter_overrides=[Parameter('mock_mode', value=True),
                                               Parameter('mock_classes', value=['cup','can'])])
    zones = ZoneService(parameter_overrides=[Parameter('mock_mode', value=True)])
    worker = AutoWorker(parameter_overrides=[Parameter('dry_step_delay', value=.01),
                                              Parameter('scan_interval', value=.05)])
    probe = Node('automatic_sort_probe')
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (vision,zones,worker,probe):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    clients = {name: probe.create_client(srv, '/worker/'+name) for name,srv in [
        ('prepare_auto_sort',PrepareAutoSort),('start_task',StartTask),
        ('session_status',SessionStatus),('task_status',TaskStatus),('cancel_task',CancelTask)]}
    cases = []
    timers = []

    def call(name, request):
        client = clients[name]
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(request)
        until(future.done, 'Service call timeout')
        return future.result()

    def prepare():
        r = call('prepare_auto_sort',PrepareAutoSort.Request())
        assert r.success, r.message
        return r

    def start(session_id, count=0):
        r = call('start_task',StartTask.Request(session_id=session_id,max_objects=count))
        assert r.accepted, r.message
        return r.task_id

    def status(task_id):
        return call('task_status',TaskStatus.Request(task_id=task_id))

    def terminal(task_id, expected):
        r = until(lambda: (s if (s:=status(task_id)).state in ('SUCCEEDED','FAILED','CANCELED') else None),
                  'Automatic task failed to terminate')
        assert r.state == expected, (r.state,r.message)
        return r

    def session(session_id):
        return call('session_status',SessionStatus.Request(session_id=session_id))

    def cleared_mock_workspace():
        assert worker.store.active is None
        # Test fixture: represent a manually cleared/restarted mock workspace.
        worker.session = None
        worker.cfg['object_size'] = .06
        worker.cfg['dry_step_delay'] = .01
        zones.cfg['mock_mode'] = True
        vision.cfg['mock_classes'] = ['cup','can']

    try:
        vision.cfg['mock_classes'] = ['cup']
        assert not call('prepare_auto_sort',PrepareAutoSort.Request()).success
        vision.cfg['mock_classes'] = ['cup','can','spoon']
        assert not call('prepare_auto_sort',PrepareAutoSort.Request()).success
        vision.cfg['mock_classes'] = ['cup','can']
        cases.append('one or three source classes cannot prepare a two-class task')
        p = prepare()
        previous_session = p.session_id
        vision.cfg['mock_classes'] = ['cup']
        assert not call('prepare_auto_sort',PrepareAutoSort.Request()).success
        assert not call('start_task',StartTask.Request(session_id=previous_session)).accepted
        vision.cfg['mock_classes'] = ['cup','can']
        p = prepare()
        cases[0] += '; failed rescan invalidates old unused session'
        mapping = {r.object_class:r.zone_id for r in p.rules}
        assert set(mapping) == {'cup','can'} and len(set(mapping.values())) == 2
        from ai_worker_interfaces.msg import DetectedObject
        placed = DetectedObject(class_name='spoon', confidence=.99)
        placed.bbox.center.position.x, placed.bbox.center.position.y = 500., 90.
        assert worker.source_objects([placed], zones.mock_zones()) == []
        cases.append('objects inside destination zones are excluded from source picking')
        vision.cfg['mock_classes'] = ['can','cup']
        terminal(start(p.session_id,1),'SUCCEEDED')
        assert session(p.session_id).placed_count == 1
        assert not call('prepare_auto_sort',PrepareAutoSort.Request()).success
        terminal(start(p.session_id),'SUCCEEDED')
        result = session(p.session_id)
        assert result.placed_count == 4
        assert {r.object_class:r.zone_id for r in result.rules} == mapping
        assert sum(s['state']=='OCCUPIED' for slots in json.loads(result.slots_json).values() for s in slots) == 4
        terminal(start(p.session_id),'SUCCEEDED')
        assert session(p.session_id).placed_count == 4
        cases.append('unknown-in-configuration cup/can auto-assigned, stable mapping, four unique slots, no repeat picks')

        cleared_mock_workspace()
        worker.cfg['object_size'] = .21
        p = prepare()
        r = terminal(start(p.session_id),'FAILED')
        assert 'ZONE_FULL' in r.message and session(p.session_id).placed_count == 1
        cases.append('full assigned zone stops task without remapping')

        cleared_mock_workspace()
        p = prepare()
        vision.cfg['mock_classes'] = ['cup','can','spoon']
        r = terminal(start(p.session_id),'FAILED')
        assert 'NEW_CLASS' in r.message and session(p.session_id).placed_count == 0
        cases.append('third class stops without changing the two-class assignment')

        cleared_mock_workspace()
        worker.cfg['dry_step_delay'] = .3
        p = prepare()
        task_id = start(p.session_id)
        assert not call('start_task',StartTask.Request(session_id=p.session_id)).accepted
        until(lambda:status(task_id).stage=='APPROACH','No approach stage')
        assert call('cancel_task',CancelTask.Request(task_id=task_id)).accepted
        terminal(task_id,'CANCELED')
        slot_states = json.loads(session(p.session_id).slots_json)
        assert any(s['state']=='UNKNOWN' for slots in slot_states.values() for s in slots)
        cases.append('busy rejection and cancellation preserve unknown slot occupancy')

        cleared_mock_workspace()
        zones.cfg['mock_mode'] = False
        image = np.full((360,640,3),220,dtype=np.uint8)
        cv2.rectangle(image,(400,20),(620,160),(0,0,255),8)
        cv2.rectangle(image,(400,200),(620,340),(255,0,0),8)
        msg = CvBridge().cv2_to_imgmsg(image,encoding='bgr8')
        msg.header.frame_id = 'test_camera_optical'
        msg.header.stamp = probe.get_clock().now().to_msg()
        image_pub = probe.create_publisher(Image,'/laptop_camera/image_raw',qos_profile_sensor_data)
        until(lambda:image_pub.get_subscription_count()>0,'Camera QoS is incompatible')
        def publish_image():
            msg.header.stamp = probe.get_clock().now().to_msg()
            image_pub.publish(msg)
        image_timer = probe.create_timer(.1,publish_image)
        timers.append(image_timer)
        until(lambda:zones.image is not None,'Best-effort image was not received')
        image_timer.cancel()
        original = bytes(zones.image.data)
        p = prepare()
        assert bytes(zones.image.data) == original, 'Overlay modified the cached camera image'
        r = call('start_task',StartTask.Request(session_id=p.session_id))
        assert not r.accepted and '2D preview' in r.message
        cases.append('actual HSV color-tape detection prepares mapping; missing metric depth blocks motion')

        cleared_mock_workspace()
        p = prepare()
        worker.cfg['dry_run'] = False
        assert not call('start_task',StartTask.Request(session_id=p.session_id)).accepted
        worker.cfg['dry_run'] = True
        cases.append('unconfigured shared grasp profile blocks hardware execution')

        # Verify visual placement logic with fabricated metric observations only.
        from types import SimpleNamespace
        from ai_worker_sorting.core import Point
        slot = next(iter(worker.session.slots.values()))[0]
        destination = DetectedObject(class_name='cup', confidence=1., localized=True, simulated=False)
        destination.surface_pose.header.frame_id = 'base_link'
        destination.surface_pose.pose.position.x = slot.position.x
        destination.surface_pose.pose.position.y = slot.position.y
        destination.surface_pose.pose.position.z = .15
        source = DetectedObject(class_name='cup', confidence=1., localized=True, simulated=False)
        source.surface_pose.header.frame_id = 'base_link'
        source.surface_pose.pose.position.x, source.surface_pose.pose.position.y, source.surface_pose.pose.position.z = .2,-.05,.15
        actual_call = worker.call
        include_source = [False]
        def observation(*a,**kw):
            stamp = probe.get_clock().now().to_msg()
            destination.surface_pose.header.stamp = source.surface_pose.header.stamp = stamp
            return SimpleNamespace(header=destination.surface_pose.header,
                                   objects=[destination,source] if include_source[0] else [destination])
        worker.call = observation
        try:
            worker.verify_placement(destination,slot,Point(.2,-.05,.15),lambda:None)
            include_source[0] = True
            worker.cfg['service_timeout'] = .1
            try:
                worker.verify_placement(destination,slot,Point(.2,-.05,.15),lambda:None)
            except ValueError as exc:
                assert 'PLACEMENT_UNVERIFIED' in str(exc)
            else:
                raise AssertionError('Source object was not removed')
        finally:
            worker.call = actual_call
        cases.append('placement verification requires destination presence and source removal')

        report = {'result':'PASS','mode':'synthetic_camera_and_dry_robot',
                  'example_mapping':mapping,'cases':cases}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))
    finally:
        for timer in timers:
            timer.cancel()
        worker.shutdown_event.set()
        if worker.thread:
            worker.thread.join(timeout=3)
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for node in (probe,worker,zones,vision):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
