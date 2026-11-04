from pathlib import Path
import unittest
import tempfile
import yaml
from ai_worker_sorting.terminal_menu import TerminalMenu


class Future:
    def __init__(self,done=True):
        self.ready = done
    def done(self):
        return self.ready


class Stack:
    mode = 'live'
    stopped = False
    def alive(self):
        return not self.stopped
    def stop(self):
        self.stopped = True


class Backend:
    def __init__(self):
        self.starts = 0
        self.state = 'RUNNING'
        self.future = Future()
    def request(self,name,**kwargs):
        if name=='prepare_auto_sort':
            return dict(success=True,session_id='session',rules=[dict(zone_id='red_zone'),dict(zone_id='blue_zone')])
        if name=='task_status':
            return dict(found=True,state=self.state,stage='TRANSFER')
        if name=='session_status':
            return dict(found=True,placed_count=1)
        return dict(accepted=True)
    def begin(self,name,**kwargs):
        self.starts += 1
        return self.future
    def result(self,future,timeout=15):
        if not future.done():
            raise TimeoutError('timeout')
        return dict(accepted=True,task_id='task')


class MenuTests(unittest.TestCase):
    def test_wizard_uses_model_and_skips_table_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            weights = root/'model.pt'
            weights.write_bytes(b'test')
            replies = iter([str(weights),'.1,.7,-.4,.4,.05,.8','cpu','','','',''])
            menu = TerminalMenu(root/'live.local.yaml',Backend(),lambda _:next(replies),lambda _:None,Stack())
            menu.launch = lambda *args:setattr(menu.stack,'stopped',False)
            calls = []
            def command(module,stage,*args,**kwargs):
                calls.append(stage)
                if stage=='scan':
                    Path(args[args.index('--state')+1]).write_text(yaml.safe_dump({'samples':{'a':[],'b':[]}}))
                elif stage=='write':
                    Path(args[args.index('--output')+1]).write_text('model: fixture')
                return ''
            menu.command = command
            menu.wizard()
            self.assertEqual(calls,['scan','grasp','grasp','write'])
            self.assertTrue(menu.config.exists())

    def menu(self,answers=()):
        self.backend,self.stack,self.output = Backend(),Stack(),[]
        responses = iter(answers)
        return TerminalMenu(Path('unused.local.yaml'),self.backend,lambda _:next(responses),self.output.append,self.stack)

    def test_launch_or_declining_start_never_sends_motion(self):
        menu = self.menu(['','n'])
        self.assertEqual(self.backend.starts,0)
        self.assertFalse(menu.start())
        self.assertEqual(self.backend.starts,0)
        self.assertTrue(any('팔을 작업물에서 떨어뜨린' in line for line in self.output))

    def test_ids_are_internal_and_only_confirmed_start_is_sent(self):
        menu = self.menu(['2','y'])
        self.assertTrue(menu.start())
        self.assertEqual((menu.session,menu.task),('session','task'))
        self.assertEqual(self.backend.starts,1)
        self.assertTrue(any('첫 번째 종류' in line for line in self.output))

    def test_unknown_start_blocks_repeat_and_process_shutdown(self):
        menu = self.menu(['2','y'])
        self.backend.future.ready = False
        with self.assertRaises(TimeoutError):
            menu.start()
        with self.assertRaises(RuntimeError):
            menu.start()
        self.assertFalse(menu.exit())
        self.assertFalse(self.stack.stopped)
        self.assertEqual(self.backend.starts,1)
        self.backend.future.ready = True
        menu.resolve_pending()
        self.assertEqual(menu.task,'task')

    def test_running_exit_is_refused_and_cancel_keeps_status(self):
        menu = self.menu()
        menu.task = 'task'
        self.assertFalse(menu.exit())
        menu.cancel()
        self.assertEqual(menu.state,'RUNNING')
        self.assertFalse(self.stack.stopped)

    def test_start_transport_exception_blocks_retries(self):
        menu = self.menu(['2','y'])
        def broken(*args,**kwargs):
            self.backend.starts += 1
            raise RuntimeError('transport disconnected')
        self.backend.begin = broken
        with self.assertRaises(RuntimeError):
            menu.start()
        with self.assertRaises(RuntimeError):
            menu.start()
        self.assertEqual(self.backend.starts,1)
        self.assertFalse(menu.exit())

    def test_failed_task_requires_physical_stop_confirmation(self):
        menu = self.menu(['n'])
        menu.task = 'task'
        self.backend.state = 'FAILED'
        self.assertFalse(menu.exit())
        self.assertFalse(self.stack.stopped)

    def test_completed_task_can_exit(self):
        menu = self.menu()
        menu.task = 'task'
        self.backend.state = 'SUCCEEDED'
        self.assertTrue(menu.exit())
        self.assertTrue(self.stack.stopped)


if __name__=='__main__':
    unittest.main()
