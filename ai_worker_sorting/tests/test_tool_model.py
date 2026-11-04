import tempfile
from pathlib import Path
import unittest
from ai_worker_sorting.tool_model import tool_extent


MODEL = '''<robot name="hand">
<link name="mount"/><link name="tcp"/>
<joint name="tcp_mount" type="fixed"><parent link="mount"/><child link="tcp"/><origin xyz="0 0 .01"/></joint>
<link name="finger"><collision><origin xyz="0 0 .04"/><geometry><box size=".02 .02 .04"/></geometry></collision></link>
<joint name="gripper" type="revolute"><parent link="mount"/><child link="finger"/>
<axis xyz="0 1 0"/><limit lower="0" upper="1.57"/></joint></robot>'''


class ToolModelTests(unittest.TestCase):
    def test_mount_tcp_offset_and_full_joint_travel(self):
        extent = tool_extent(MODEL,'tcp','gripper',[1.,0.,0.,0.])
        self.assertGreaterEqual(extent,.053-1e-9)
        self.assertLess(extent,.08)

    def test_missing_model_link_never_uses_default_length(self):
        with self.assertRaises(ValueError):
            tool_extent(MODEL,'missing','gripper',[1.,0.,0.,0.])

    def test_articulated_tcp_is_rejected(self):
        with self.assertRaises(ValueError):
            tool_extent(MODEL.replace('name="tcp_mount" type="fixed"','name="tcp_mount" type="revolute"'),
                        'tcp','gripper',[1.,0.,0.,0.])

    def test_missing_mesh_asset_is_rejected(self):
        with self.assertRaises(FileNotFoundError):
            tool_extent(MODEL.replace('<box size=".02 .02 .04"/>','<mesh filename="/missing/finger.stl"/>'),
                        'tcp','gripper',[1.,0.,0.,0.])

    def test_local_stl_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'finger.stl'
            path.write_text('solid f\n vertex 0 0 0\n vertex .01 0 .02\n vertex 0 .01 .02\nendsolid f\n')
            xml = MODEL.replace('<box size=".02 .02 .04"/>',f'<mesh filename="{path.as_posix()}"/>')
            self.assertGreater(tool_extent(xml,'tcp','gripper',[1.,0.,0.,0.]),.05)


if __name__=='__main__':
    unittest.main()
