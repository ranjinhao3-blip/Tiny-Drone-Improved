"""无人机传感器只读监视器。连接无人机 Wi-Fi 后运行，不发送飞行控制命令。"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.error import URLError
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener


DEFAULT_URL = 'http://192.168.26.28/api/poshold'
PAGE = Path(__file__).with_name('sensor_monitor.html')


def make_server(url=DEFAULT_URL, port=8765, timeout=1.5):
    """构造仅监听本机的服务，便于现场运行和离线验证。"""
    opener = build_opener(ProxyHandler({}))
    upstream_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, body, content_type):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # 关闭页面不影响下次读取。

        def do_GET(self):
            path = urlsplit(self.path).path
            if path in ('/', '/sensor_monitor.html'):
                self.reply(200, PAGE.read_bytes(), 'text/html; charset=utf-8')
            elif path == '/snapshot':
                try:
                    request = Request(url, headers={'Cache-Control': 'no-cache',
                                                    'Accept': 'application/json'})
                    # 多个页面同时打开时，也不并发请求机载 HTTP 服务。
                    with upstream_lock:
                        with opener.open(request, timeout=timeout) as response:
                            data = json.loads(response.read().decode('utf-8-sig'))
                    if not isinstance(data, dict):
                        raise ValueError('无人机返回的数据不是 JSON 对象')
                    body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
                    self.reply(200, body, 'application/json; charset=utf-8')
                except (URLError, OSError, ValueError, UnicodeError) as error:
                    body = json.dumps({'error': str(error)}, ensure_ascii=False).encode('utf-8')
                    self.reply(502, body, 'application/json; charset=utf-8')
            else:
                self.send_error(404)

        def log_message(self, fmt, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default=DEFAULT_URL, help='无人机诊断接口地址')
    parser.add_argument('--port', type=int, default=8765, help='本机网页端口，默认 8765')
    args = parser.parse_args()
    parsed = urlsplit(args.url)
    if parsed.scheme not in ('http', 'https') or not parsed.netloc:
        parser.error('--url 必须是完整的 http:// 或 https:// 地址')
    if not 1 <= args.port <= 65535:
        parser.error('--port 必须在 1～65535 之间')
    try:
        server = make_server(args.url, args.port)
    except OSError as error:
        parser.exit(1, f'无法启动监视器：{error}\n端口可能已被占用，可以用 --port 8766。\n')
    print(f'传感器监视器：http://127.0.0.1:{args.port}/', flush=True)
    print(f'只读数据源：{args.url}\n请连接无人机 Wi-Fi；按 Ctrl+C 退出。', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n监视器已停止。')
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
