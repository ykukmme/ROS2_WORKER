"""Controller lifecycle regression without ROS installation or robot access."""
from concurrent.futures import Future
import importlib.util
from pathlib import Path
import threading
import types
import unittest
from unittest.mock import Mock, patch


def adapter_type():
    modules = {}
    for name, symbols in {
        'action_msgs.msg': ['GoalStatus'], 'control_msgs.action': ['GripperCommand'],
        'geometry_msgs.msg': ['Pose'], 'moveit_msgs.action': ['MoveGroup'],
        'moveit_msgs.msg': ['Constraints','PositionConstraint','OrientationConstraint'],
        'rclpy.action': ['ActionClient'], 'shape_msgs.msg': ['SolidPrimitive'],
        'std_srvs.srv': ['Trigger'],
    }.items():
        module = types.ModuleType(name)
        for symbol in symbols:
            setattr(module,symbol,type(symbol,(),{}))
        modules[name] = module
    modules['action_msgs.msg'].GoalStatus.STATUS_SUCCEEDED = 4
    path = Path(__file__).parents[1]/'ai_worker_sorting'/'robot_interface.py'
    spec = importlib.util.spec_from_file_location('robot_cleanup_test_adapter',path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules',modules):
        spec.loader.exec_module(module)
    return module.MoveItRobot


class ActionCleanupTests(unittest.TestCase):
    def make_adapter(self):
        adapter = object.__new__(adapter_type())
        adapter.node = Mock()
        adapter.cfg = {'action_timeout':1.0}
        adapter._handles, adapter._lock = [],threading.Lock()
        adapter._failed_action = False
        adapter.move_client,adapter.grip_client,adapter.verify_client = Mock(),Mock(),Mock()
        return adapter

    def test_canceled_pending_goal_retains_clients_for_late_acceptance(self):
        adapter = self.make_adapter()
        # Mock has dynamic attributes; the real ROS Node does not.
        adapter.node._failed_robot_adapters = []
        pending = Future()
        adapter.move_client.send_goal_async.return_value = pending
        adapter.move_client.wait_for_server.return_value = True
        calls = [0]
        def check():
            calls[0] += 1
            if calls[0] > 1:
                raise RuntimeError('cancel requested')
        adapter.check = check
        with self.assertRaisesRegex(RuntimeError,'cancel requested'):
            adapter.action(adapter.move_client,object())
        adapter.destroy()
        adapter.move_client.destroy.assert_not_called()
        adapter.grip_client.destroy.assert_not_called()
        self.assertIn(adapter,adapter.node._failed_robot_adapters)
        late_handle = Mock(accepted=True)
        pending.set_result(late_handle)
        late_handle.cancel_goal_async.assert_called_once()

    def test_successful_cleanup_releases_clients(self):
        adapter = self.make_adapter()
        adapter.destroy()
        adapter.move_client.destroy.assert_called_once()
        adapter.grip_client.destroy.assert_called_once()
        adapter.node.destroy_client.assert_called_once_with(adapter.verify_client)


if __name__=='__main__':
    unittest.main()
