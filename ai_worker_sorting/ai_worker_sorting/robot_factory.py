"""Select a concrete controller adapter without importing unused robot stacks."""


def make_robot(node, cfg, check, grasp_verifier=None):
    backend = cfg.get('robot_backend', 'moveit')
    if backend == 'robotis_movel':
        from .robotis_robot import RobotisMoveLRobot
        return RobotisMoveLRobot(node, cfg, check, grasp_verifier)
    if backend == 'moveit':
        from .robot_interface import MoveItRobot
        return MoveItRobot(node, cfg, check, grasp_verifier)
    raise ValueError(f'Unknown robot_backend: {backend}')
