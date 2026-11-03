import threading
import unittest
from ai_worker_sorting.tasks import TaskStore


class TaskStoreTests(unittest.TestCase):
    def test_busy_and_terminal_release(self):
        store = TaskStore()
        task = store.admit('box')
        with self.assertRaises(RuntimeError):
            store.admit('bottle')
        store.finish(task, 'SUCCEEDED', 'done')
        self.assertNotEqual(store.admit('bottle').task_id, task.task_id)

    def test_simultaneous_requests_admit_exactly_one(self):
        store, results = TaskStore(), []
        barrier = threading.Barrier(8)
        def admit():
            barrier.wait()
            try:
                results.append(store.admit('box'))
            except RuntimeError:
                pass
        threads = [threading.Thread(target=admit) for _ in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(len(results), 1)

    def test_cancel_is_pending_until_worker_finishes(self):
        store = TaskStore()
        task = store.admit('box')
        self.assertTrue(store.request_cancel(task.task_id))
        self.assertEqual(store.snapshot(task.task_id)['state'], 'QUEUED')
        self.assertTrue(task.cancel.is_set())
        with self.assertRaises(RuntimeError):
            store.admit('box')
        store.finish(task, 'CANCELED', 'stopped')
        self.assertFalse(store.request_cancel(task.task_id))

    def test_completion_cancel_race_never_reports_success(self):
        store = TaskStore()
        task = store.admit('box')
        store.request_cancel(task.task_id)
        store.finish(task, 'SUCCEEDED', 'done')
        self.assertEqual(store.snapshot(task.task_id)['state'], 'CANCELED')

    def test_hardware_fault_blocks_new_jobs(self):
        store = TaskStore()
        task = store.admit('box')
        store.finish(task, 'FAILED', 'uncertain controller state', fault=True)
        with self.assertRaises(RuntimeError):
            store.admit('box')

    def test_history_eviction_and_unknown_task(self):
        store = TaskStore(capacity=2)
        tasks = []
        for _ in range(3):
            task = store.admit('box')
            store.finish(task, 'SUCCEEDED', 'done')
            tasks.append(task)
        self.assertIsNone(store.snapshot(tasks[0].task_id))
        self.assertFalse(store.request_cancel('unknown'))
        self.assertEqual(len(store.tasks), 2)


if __name__ == '__main__':
    unittest.main()
