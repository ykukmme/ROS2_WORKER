"""Docker process fixtures; no hardware calls. Run inside the built image.

`pty`: checks interactive menu start/exit.
`eof`: keeps an owned sleep process alive after stdin EOF, until a test-only
       /data/recovered marker changes the emulated status to SUCCEEDED.
"""
import argparse
import os
from pathlib import Path
import pty
import select
import subprocess
import time
from ai_worker_sorting.terminal_menu import TerminalMenu,ManagedStack


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['pty','eof'])
    args = parser.parse_args()
    if args.mode=='eof':
        class Backend:
            def request(self,name,**values):
                if name=='cancel_task':
                    return {'accepted':True}
                return {'found':True,'state':'SUCCEEDED' if Path('/data/recovered').exists() else 'FAILED','stage':'HOLD'}
        stack = ManagedStack(Path('/data/probe-logs'))
        stack.start('live',['sleep','300'])
        Path('/data/worker.pid').write_text(str(stack.process.pid))
        menu = TerminalMenu(Path('/data/live.local.yaml'),Backend(),stack=stack)
        menu.task = 'emulated-task'
        return menu.run()
    master,slave = pty.openpty()
    child = subprocess.Popen(['/opt/worker/entrypoint.sh','menu'],stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
    os.close(slave)
    output = b''
    try:
        deadline = time.monotonic()+20
        while time.monotonic()<deadline:
            if select.select([master],[],[],.2)[0]:
                output += os.read(master,65536)
                if '선택:'.encode() in output:
                    break
        else:
            raise AssertionError(output.decode(errors='replace'))
        os.write(master,b'0\n')
        assert child.wait(timeout=10)==0
        print('Interactive Docker menu start/exit PASS')
        return 0
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=5)
        os.close(master)


if __name__=='__main__':
    raise SystemExit(main())
