"""Exercise the local read-only proxy with a fake aircraft; no aircraft needed."""
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import sys
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

sys.dont_write_bytecode = True
from monitor_sensors import make_server


class SensorMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.body = b'{"stateValid":true,"flow":{"dx":0},"sensors":{"available":true}}'
        cls.paths = []
        cls.active = 0
        cls.peak = 0

        class Aircraft(BaseHTTPRequestHandler):
            def do_GET(self):
                cls.paths.append(self.path)
                cls.active += 1
                cls.peak = max(cls.peak, cls.active)
                try:
                    time.sleep(.02)
                    body = cls.body
                    self.send_response(200)
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                finally:
                    cls.active -= 1

            def log_message(self, *args):
                pass

        cls.aircraft = ThreadingHTTPServer(('127.0.0.1', 0), Aircraft)
        cls.proxy = make_server(f'http://127.0.0.1:{cls.aircraft.server_port}/api/poshold', port=0)
        cls.base = f'http://127.0.0.1:{cls.proxy.server_port}'
        cls.opener = build_opener(ProxyHandler({}))
        for server in (cls.aircraft, cls.proxy):
            threading.Thread(target=server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        for server in (cls.proxy, cls.aircraft):
            server.shutdown()
            server.server_close()

    def read(self, path='/snapshot'):
        with self.opener.open(self.base + path, timeout=2) as response:
            return response.status, response.headers, response.read()

    def test_page_and_json(self):
        status, headers, body = self.read('/?test=1')
        self.assertEqual(status, 200)
        self.assertIn('text/html', headers['Content-Type'])
        self.assertIn('无人机传感器监视器', body.decode())
        status, headers, body = self.read()
        self.assertEqual(status, 200)
        self.assertEqual(headers['Cache-Control'], 'no-store')
        self.assertTrue(json.loads(body)['sensors']['available'])
        self.assertEqual(self.paths[-1], '/api/poshold')

    def test_bad_json_and_non_object_return_json_error(self):
        original = type(self).body
        try:
            for body in (b'not json', b'[]', b'{"velocity":[NaN]}'):
                type(self).body = body
                with self.assertRaises(HTTPError) as caught:
                    self.read()
                self.assertEqual(caught.exception.code, 502)
                self.assertIn('error', json.loads(caught.exception.read()))
        finally:
            type(self).body = original

    def test_read_only_routes(self):
        count = len(self.paths)
        for request, expected in ((Request(self.base + '/snapshot', method='POST'), 501),
                                  (Request(self.base + '/api/control'), 404)):
            with self.assertRaises(HTTPError) as caught:
                self.opener.open(request)
            self.assertEqual(caught.exception.code, expected)
        self.assertEqual(len(self.paths), count)

    def test_upstream_requests_are_serialized(self):
        type(self).peak = 0
        with ThreadPoolExecutor(max_workers=3) as pool:
            results = list(pool.map(lambda _: self.read(), range(3)))
        self.assertTrue(all(status == 200 for status, _, _ in results))
        self.assertEqual(type(self).peak, 1)


if __name__ == '__main__':
    unittest.main()
