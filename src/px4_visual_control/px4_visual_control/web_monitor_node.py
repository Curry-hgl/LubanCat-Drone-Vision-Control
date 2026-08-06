#!/usr/bin/env python3

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State, RCOut


HTML_PAGE = r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <title>MAVROS 状态监控</title>

  <style>
    body {
      margin: 0;
      padding: 24px;
      color: #e5e7eb;
      background: #111827;
      font-family: Arial, sans-serif;
    }

    h1 {
      margin-top: 0;
    }

    .status {
      margin-bottom: 20px;
      padding: 12px 16px;
      border-radius: 8px;
      background: #1f2937;
    }

    .grid {
      display: grid;
      grid-template-columns:
        repeat(auto-fit, minmax(180px, 1fr));
      gap: 16px;
    }

    .card {
      padding: 18px;
      border-radius: 10px;
      background: #1f2937;
    }

    .label {
      color: #9ca3af;
      font-size: 14px;
    }

    .value {
      margin-top: 8px;
      font-size: 25px;
      font-weight: bold;
    }

    .true {
      color: #22c55e;
    }

    .false {
      color: #ef4444;
    }

    .motor-grid {
      display: grid;
      grid-template-columns:
        repeat(auto-fit, minmax(150px, 1fr));
      gap: 12px;
    }

    .motor {
      padding: 14px;
      border-radius: 8px;
      background: #374151;
    }

    .bar-background {
      width: 100%;
      height: 12px;
      margin-top: 10px;
      overflow: hidden;
      border-radius: 6px;
      background: #111827;
    }

    .bar {
      height: 100%;
      width: 0%;
      background: #3b82f6;
      transition: width 0.15s;
    }

    .timestamp {
      margin-top: 18px;
      color: #9ca3af;
      font-size: 13px;
    }
  </style>
</head>

<body>
  <h1>MAVROS 状态监控</h1>

  <div id="connection" class="status">
    正在连接鲁班猫……
  </div>

  <div class="grid">
    <div class="card">
      <div class="label">FCU连接</div>
      <div id="connected" class="value">--</div>
    </div>

    <div class="card">
      <div class="label">解锁状态</div>
      <div id="armed" class="value">--</div>
    </div>

    <div class="card">
      <div class="label">飞行模式</div>
      <div id="mode" class="value">--</div>
    </div>

    <div class="card">
      <div class="label">系统状态</div>
      <div id="system-status" class="value">--</div>
    </div>

    <div class="card">
      <div class="label">Guided标志</div>
      <div id="guided" class="value">--</div>
    </div>

    <div class="card">
      <div class="label">遥控输入</div>
      <div id="manual-input" class="value">--</div>
    </div>
  </div>

  <h2>电机/舵机输出</h2>

  <div id="motors" class="motor-grid"></div>

  <div id="timestamp" class="timestamp"></div>

  <script>
    function showBoolean(elementId, value) {
      const element = document.getElementById(elementId);

      if (value === null || value === undefined) {
        element.textContent = '--';
        element.className = 'value';
        return;
      }

      element.textContent = value ? 'TRUE' : 'FALSE';
      element.className = value ? 'value true' : 'value false';
    }

    function pwmPercent(value) {
      if (!value || value <= 0) {
        return 0;
      }

      return Math.max(
        0,
        Math.min(100, (value - 1000) / 10)
      );
    }

    function renderMotors(channels) {
      const container = document.getElementById('motors');
      container.innerHTML = '';

      if (!channels || channels.length === 0) {
        container.innerHTML =
          '<div class="card">暂无 RC OUT 数据</div>';
        return;
      }

      channels.forEach((pwm, index) => {
        const motor = document.createElement('div');
        motor.className = 'motor';

        const percent = pwmPercent(pwm);

        motor.innerHTML = `
          <div class="label">输出通道 ${index + 1}</div>
          <div class="value">${pwm}</div>
          <div class="bar-background">
            <div class="bar" style="width:${percent}%"></div>
          </div>
        `;

        container.appendChild(motor);
      });
    }

    async function updateStatus() {
      try {
        const response = await fetch(
          '/api/status',
          {cache: 'no-store'}
        );

        const data = await response.json();

        document.getElementById('connection').textContent =
          'Web服务连接正常';

        showBoolean('connected', data.state.connected);
        showBoolean('armed', data.state.armed);
        showBoolean('guided', data.state.guided);
        showBoolean(
          'manual-input',
          data.state.manual_input
        );

        document.getElementById('mode').textContent =
          data.state.mode || '--';

        document.getElementById('system-status').textContent =
          data.state.system_status ?? '--';

        renderMotors(data.rc_out.channels);

        document.getElementById('timestamp').textContent =
          '页面更新时间：' + new Date().toLocaleString();

      } catch (error) {
        document.getElementById('connection').textContent =
          '无法连接鲁班猫 Web 服务';
      }
    }

    updateStatus();
    setInterval(updateStatus, 250);
  </script>
</body>
</html>
'''


class WebMonitorNode(Node):

    def __init__(self):
        super().__init__('mavros_web_monitor')

        self.declare_parameter('port', 8080)
        self.port = int(self.get_parameter('port').value)

        self.data_lock = threading.Lock()

        self.state_data = {
            'received': False,
            'connected': False,
            'armed': False,
            'guided': False,
            'manual_input': False,
            'mode': '',
            'system_status': 0,
        }

        self.rc_out_data = {
            'received': False,
            'channels': [],
        }

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.rc_out_sub = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.rc_out_callback,
            10
        )

        node = self

        class RequestHandler(BaseHTTPRequestHandler):

            def do_GET(self):
                if self.path == '/' or self.path == '/index.html':
                    content = HTML_PAGE.encode('utf-8')

                    self.send_response(200)
                    self.send_header(
                        'Content-Type',
                        'text/html; charset=utf-8'
                    )
                    self.send_header(
                        'Content-Length',
                        str(len(content))
                    )
                    self.end_headers()
                    self.wfile.write(content)
                    return

                if self.path.startswith('/api/status'):
                    with node.data_lock:
                        response_data = {
                            'state': dict(node.state_data),
                            'rc_out': {
                                'received':
                                    node.rc_out_data['received'],
                                'channels':
                                    list(
                                        node.rc_out_data['channels']
                                    ),
                            },
                        }

                    content = json.dumps(
                        response_data
                    ).encode('utf-8')

                    self.send_response(200)
                    self.send_header(
                        'Content-Type',
                        'application/json'
                    )
                    self.send_header(
                        'Cache-Control',
                        'no-store'
                    )
                    self.send_header(
                        'Content-Length',
                        str(len(content))
                    )
                    self.end_headers()
                    self.wfile.write(content)
                    return

                self.send_response(404)
                self.end_headers()

            def log_message(self, format_string, *args):
                # 禁止每次网页刷新都向终端打印日志
                return

        self.http_server = ThreadingHTTPServer(
            ('0.0.0.0', self.port),
            RequestHandler
        )

        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True
        )
        self.http_thread.start()

        self.get_logger().info(
            f'Web监控已启动：http://0.0.0.0:{self.port}'
        )

    def state_callback(self, msg):
        with self.data_lock:
            self.state_data = {
                'received': True,
                'connected': bool(msg.connected),
                'armed': bool(msg.armed),
                'guided': bool(msg.guided),
                'manual_input': bool(msg.manual_input),
                'mode': str(msg.mode),
                'system_status': int(msg.system_status),
            }

    def rc_out_callback(self, msg):
        with self.data_lock:
            self.rc_out_data = {
                'received': True,
                'channels': [int(v) for v in msg.channels],
            }

    def destroy_node(self):
        self.http_server.shutdown()
        self.http_server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()