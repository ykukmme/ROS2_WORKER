"""Thread-safe, bounded in-memory job registry; no ROS dependencies."""
from collections import OrderedDict
from dataclasses import dataclass, field
import threading
import uuid

TERMINAL = frozenset({'SUCCEEDED', 'FAILED', 'CANCELED'})


class TaskCanceled(RuntimeError):
    pass


@dataclass
class Task:
    task_id: str
    target_class: str
    state: str = 'QUEUED'
    stage: str = 'QUEUED'
    message: str = ''
    cancel: threading.Event = field(default_factory=threading.Event)


class TaskStore:
    def __init__(self, capacity=100):
        if capacity < 1:
            raise ValueError('Task capacity must be positive')
        self.capacity = capacity
        self.lock = threading.RLock()
        self.tasks = OrderedDict()
        self.active = None
        self.faulted = False

    def admit(self, target_class):
        with self.lock:
            if self.active is not None or self.faulted:
                raise RuntimeError('Worker busy or hardware fault latched')
            while len(self.tasks) >= self.capacity:
                self.tasks.popitem(last=False)
            task = Task(uuid.uuid4().hex, target_class)
            self.tasks[task.task_id] = task
            self.active = task.task_id
            return task

    def snapshot(self, task_id):
        with self.lock:
            task = self.tasks.get(task_id)
            if task is None:
                return None
            return dict(task_id=task.task_id, target_class=task.target_class,
                        state=task.state, stage=task.stage, message=task.message)

    def stage(self, task, stage):
        with self.lock:
            if task.state not in TERMINAL:
                task.state, task.stage = 'RUNNING', stage

    def request_cancel(self, task_id):
        with self.lock:
            task = self.tasks.get(task_id)
            if task is None or task.state in TERMINAL:
                return False
            task.cancel.set()
            task.message = 'Cancellation requested; awaiting worker/controller outcome'
            return True

    def finish(self, task, state, message, fault=False):
        if state not in TERMINAL:
            raise ValueError('Invalid terminal state')
        with self.lock:
            if task.state in TERMINAL:
                return
            if state == 'SUCCEEDED' and task.cancel.is_set():
                state, message = 'CANCELED', 'Canceled at completion boundary'
            task.state, task.message = state, message
            if state == 'SUCCEEDED':
                task.stage = 'DONE'
            self.faulted |= fault
            if self.active == task.task_id:
                self.active = None
