from http.server import ThreadingHTTPServer
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler

from ai_worker_sorting.webcam_bridge import Frames, handler_for


class CameraProtocolTests(unittest.TestCase):
    def setUp(self):
        self.state = Frames()
        self.token = 'test-token-' + 'x'*32
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(self.state, self.token))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def request(self, path='/frame.jpg', token=None):
        headers = {} if token is None else {'X-Bridge-Token': token}
        return self.opener.open(Request(self.url+path, headers=headers), timeout=2)

    def test_unauthenticated_camera_access_rejected(self):
        with self.assertRaises(HTTPError) as caught:
            self.request()
        self.assertEqual(caught.exception.code, 401)

    def test_wrong_token_rejected(self):
        with self.assertRaises(HTTPError) as caught:
            self.request(token='wrong')
        self.assertEqual(caught.exception.code, 401)

    def test_no_frame_returns_unavailable(self):
        with self.assertRaises(HTTPError) as caught:
            self.request(token=self.token)
        self.assertEqual(caught.exception.code, 503)

    def test_frame_stamp_and_sequence_preserved(self):
        self.state.jpeg = b'jpeg-test-payload'
        self.state.stamp, self.state.sequence = 123456789, 42
        with self.request(token=self.token) as response:
            self.assertEqual(response.read(), b'jpeg-test-payload')
            self.assertEqual(response.headers['X-Capture-Nanoseconds'], '123456789')
            self.assertEqual(response.headers['X-Frame-Sequence'], '42')
            self.assertEqual(response.headers['Cache-Control'], 'no-store')

    def test_arbitrary_paths_unavailable(self):
        with self.assertRaises(HTTPError) as caught:
            self.request('/other', token=self.token)
        self.assertEqual(caught.exception.code, 404)


if __name__ == '__main__':
    unittest.main()
