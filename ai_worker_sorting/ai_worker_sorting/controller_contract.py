"""Read the running Cyclo controller contract before allowing commands."""
import time
from rcl_interfaces.srv import GetParameters


def verify_controller(node, cfg, spin=False):
    import rclpy
    client = node.create_client(GetParameters, cfg['controller_node'].rstrip('/')+'/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Cyclo controller parameter service unavailable')
        names = ['l_gripper_name','left_movel_topic','left_raw_traj_topic',
                 'left_gripper_joint','joint_states_topic']
        future = client.call_async(GetParameters.Request(names=names))
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic()<deadline:
            if spin:
                rclpy.spin_once(node,timeout_sec=.02)
            else:
                time.sleep(.02)
        if not future.done() or future.result() is None:
            raise RuntimeError('Cyclo controller parameters unavailable')
        values = future.result().values
        if len(values)!=len(names) or any(v.type!=4 or not v.string_value for v in values):
            raise RuntimeError('Unsupported Cyclo controller parameters')
        actual = dict(zip(names,[v.string_value for v in values]))
        expected = dict(left_movel_topic=cfg['movel_topic'],
                        left_raw_traj_topic=cfg['gripper_trajectory_topic'],
                        left_gripper_joint=cfg['gripper_joint'],joint_states_topic=cfg['joint_states_topic'])
        if any(node.resolve_topic_name(actual[k])!=node.resolve_topic_name(v) for k,v in expected.items()):
            raise RuntimeError('Cyclo topics/joint do not match the configured contract')
        if cfg.get('tool_link') and cfg['tool_link']!=actual['l_gripper_name']:
            raise RuntimeError('TCP tool_link must equal the actual Cyclo l_gripper_name')
        return actual['l_gripper_name']
    finally:
        node.destroy_client(client)
