import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import yaml
from ai_worker_sorting.runtime_defaults import inference_device,model_path
from ai_worker_sorting.terminal_menu import TerminalMenu
from test_terminal_menu import Backend,Stack


class RuntimeDefaultsTests(unittest.TestCase):
    def test_disconnected_container_keeps_worker_until_confirmed(self):
        backend,stack = Backend(),Stack()
        menu = TerminalMenu(Path('unused.local.yaml'),backend,show=lambda _:None,stack=stack)
        menu.task = 'task'
        def recover(_):
            self.assertFalse(stack.stopped)
            backend.state = 'SUCCEEDED'
        with patch('ai_worker_sorting.terminal_menu.time.sleep',side_effect=recover):
            self.assertEqual(menu.disconnected(),2)
        self.assertTrue(stack.stopped)

    def test_auto_gpu_and_cpu(self):
        self.assertEqual(inference_device('auto',True),'0')
        self.assertEqual(inference_device('auto',False),'cpu')
        self.assertEqual(inference_device('cpu',True),'cpu')

    def test_missing_saved_model_never_falls_back_to_different_classes(self):
        with tempfile.TemporaryDirectory() as directory:
            bundled = Path(directory)/'bundled.pt'
            bundled.write_bytes(b'fixture')
            with patch.dict('os.environ',{'WORKER_DEFAULT_WEIGHTS':str(bundled)}):
                self.assertEqual(model_path(),bundled)
                with self.assertRaises(ValueError):
                    model_path(str(Path(directory)/'missing-custom.pt'))

    def test_bundled_wizard_prompts_only_workspace_and_teaching(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            weights = root/'bundled.pt'
            weights.write_bytes(b'fixture')
            prompts,calls = [],[]
            def answer(prompt):
                prompts.append(prompt)
                return '.1,.7,-.4,.4,.05,.8' if '작업 범위' in prompt else ''
            menu = TerminalMenu(root/'live.local.yaml',Backend(),answer,lambda _:None,Stack())
            menu.launch = lambda *args:setattr(menu.stack,'stopped',False)
            def command(module,stage,*args,**kwargs):
                calls.append(stage)
                if stage=='scan':
                    Path(args[args.index('--state')+1]).write_text(yaml.safe_dump({'samples':{'a':[],'b':[]}}))
                elif stage=='write':
                    Path(args[args.index('--output')+1]).write_text('test: complete')
            menu.command = command
            with patch.dict('os.environ',{'WORKER_DEFAULT_WEIGHTS':str(weights),'WORKER_WORKSPACE':''}),patch('ai_worker_sorting.terminal_menu.inference_device',return_value='cpu'):
                menu.wizard()
            self.assertTrue(any('작업 범위' in p for p in prompts))
            self.assertFalse(any('가중치 경로' in p or '추론 장치' in p for p in prompts))
            self.assertEqual(calls,['scan','grasp','grasp','write'])

    def test_runtime_device_adjustment_preserves_saved_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            weights = root/'custom.pt'
            weights.write_bytes(b'fixture')
            config = root/'live.local.yaml'
            config.write_text(yaml.safe_dump({'rgbd_detector':{'ros__parameters':{'weights':str(weights),'device':'0'}}}))
            menu = TerminalMenu(config,Backend(),stack=Stack())
            before = config.read_bytes()
            with patch('ai_worker_sorting.terminal_menu.inference_device',return_value='cpu'):
                runtime = menu.runtime_profile()
            self.assertEqual(config.read_bytes(),before)
            self.assertEqual(yaml.safe_load(runtime.read_text())['rgbd_detector']['ros__parameters']['device'],'cpu')


if __name__=='__main__':
    unittest.main()
