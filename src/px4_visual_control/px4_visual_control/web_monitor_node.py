#!/usr/bin/env python3

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State, RCOut, AttitudeTarget

import math
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


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

    .control-panel {
      padding: 18px;
      border-radius: 10px;
      background: #1f2937;
    }

    .button-row {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      margin-top: 14px;
    }

    button {
      min-width: 100px;
      padding: 12px 18px;
      border: 0;
      border-radius: 8px;
      color: white;
      background: #2563eb;
      font-size: 16px;
      cursor: pointer;
    }

    button:hover {
      background: #1d4ed8;
    }

    #angle-step {
      width: 70px;
      padding: 7px;
      margin-left: 8px;
    }

    #command-state {
      margin-top: 18px;
      color: #fbbf24;
      font-family: monospace;
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

  <h2>飞行控制</h2>

  <div class="control-panel">
    <label>
      单次角度：
      <input
        id="angle-step"
        type="number"
        value="1.0"
        min="0.5"
        max="5.0"
        step="0.5"
      >
      °
    </label>

    <div class="button-row">
      <button onclick="sendCommand('forward')">
        前进
      </button>

      <button onclick="sendCommand('backward')">
        后退
      </button>

      <button onclick="sendCommand('up')">
        上升
      </button>

      <button onclick="sendCommand('down')">
        下降
      </button>
    </div>

    <div class="button-row">
      <button onclick="sendCommand('left')">
        左移
      </button>

      <button onclick="sendCommand('right')">
        右移
      </button>

      <button onclick="sendCommand('yaw_left')">
        左旋
      </button>

      <button onclick="sendCommand('yaw_right')">
        右旋
      </button>
    </div>

    <div class="button-row">
      <button onclick="sendCommand('level')">
        恢复水平
      </button>

      <button onclick="sendCommand('stop')">
        停止
      </button>
    </div>

    <div id="command-state">
      Roll: 0.0°，
      Pitch: 0.0°，
      Yaw: 0.0°，
      Thrust: 0.50
    </div>
  </div>

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

        renderMotors(data.rc_out.channels.slice(0, 4));

        document.getElementById('timestamp').textContent =
          '页面更新时间：' + new Date().toLocaleString();

      } catch (error) {
        document.getElementById('connection').textContent =
          '无法连接鲁班猫 Web 服务';
      }
    }

    async function sendCommand(action) {
      const input = document.getElementById('angle-step');
      const step = Number(input.value);

      try {
        const response = await fetch('/api/command', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json'
          },
          body: JSON.stringify({
            action: action,
            step: step
          })
        });

        const result = await response.json();

        if (!result.success) {
          alert('控制命令失败：' + result.error);
          return;
        }

        document.getElementById(
          'command-state'
        ).textContent =
          `Roll: ${result.roll_deg.toFixed(1)}°，` +
          `Pitch: ${result.pitch_deg.toFixed(1)}°，` +
          `Yaw: ${result.yaw_offset_deg.toFixed(1)}°，` +
          `Thrust: ${result.thrust.toFixed(2)}`;

      } catch (error) {
        alert('无法连接控制服务：' + error);
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

        self.fcu_armed = False
        self.fcu_mode = ''
        self.current_yaw = 0.0

        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_offset_deg = 0.0
        self.thrust = 0.5

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

        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )

        self.attitude_timer = self.create_timer(
            0.05,  # 20 Hz
            self.publish_attitude
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

            def do_POST(self):
                if self.path != '/api/command':
                    self.send_response(404)
                    self.end_headers()
                    return

                length = int(
                    self.headers.get('Content-Length', 0)
                )

                body = self.rfile.read(length)

                try:
                    command = json.loads(
                        body.decode('utf-8')
                    )

                    action = command.get('action')
                    step = float(
                        command.get('step', 1.0)
                    )

                    with node.data_lock:
                        if action == 'forward':
                            node.pitch_deg -= step

                        elif action == 'backward':
                            node.pitch_deg += step

                        elif action == 'left':
                            node.roll_deg -= step

                        elif action == 'right':
                            node.roll_deg += step

                        elif action == 'yaw_left':
                            node.yaw_offset_deg += step

                        elif action == 'yaw_right':
                            node.yaw_offset_deg -= step

                        elif action == 'up':
                            node.thrust += 0.01

                        elif action == 'down':
                            node.thrust -= 0.01

                        elif action == 'level':
                            node.roll_deg = 0.0
                            node.pitch_deg = 0.0

                        elif action == 'stop':
                            node.roll_deg = 0.0
                            node.pitch_deg = 0.0
                            node.yaw_offset_deg = 0.0
                            node.thrust = 0.5

                        node.roll_deg = max(
                            -5.0,
                            min(5.0, node.roll_deg)
                        )

                        node.pitch_deg = max(
                            -5.0,
                            min(5.0, node.pitch_deg)
                        )

                        node.thrust = max(
                            0.45,
                            min(0.55, node.thrust)
                        )

                    response = json.dumps({
                        'success': True,
                        'roll_deg': node.roll_deg,
                        'pitch_deg': node.pitch_deg,
                        'yaw_offset_deg': node.yaw_offset_deg,
                        'thrust': node.thrust,
                    }).encode('utf-8')

                    self.send_response(200)
                    self.send_header(
                        'Content-Type',
                        'application/json'
                    )
                    self.send_header(
                        'Content-Length',
                        str(len(response))
                    )
                    self.end_headers()
                    self.wfile.write(response)

                except Exception as exc:
                    response = json.dumps({
                        'success': False,
                        'error': str(exc),
                    }).encode('utf-8')

                    self.send_response(400)
                    self.send_header(
                        'Content-Type',
                        'application/json'
                    )
                    self.send_header(
                        'Content-Length',
                        str(len(response))
                    )
                    self.end_headers()
                    self.wfile.write(response)

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

    def rc_out_callback(self, msg):
        with self.data_lock:
            self.rc_out_data = {
                'received': True,
                'channels': [int(v) for v in msg.channels],
            }

    def state_callback(self, msg):
        self.fcu_armed = msg.armed
        self.fcu_mode = msg.mode

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

    def imu_callback(self, msg):
        q = msg.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def publish_attitude(self):
        # 不能只判断 armed。
        # 必须确认已经进入 GUIDED_NOGPS。
        if not self.fcu_armed:
            return

        if self.fcu_mode != 'GUIDED_NOGPS':
            return

        roll = math.radians(self.roll_deg)
        pitch = math.radians(self.pitch_deg)

        yaw = (
            self.current_yaw
            + math.radians(self.yaw_offset_deg)
        )

        qx, qy, qz, qw = euler_to_quaternion(
            roll,
            pitch,
            yaw
        )

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 使用四元数，忽略三个角速度
        msg.type_mask = 7

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0

        msg.thrust = max(
            0.45,
            min(0.55, self.thrust)
        )

        self.attitude_pub.publish(msg)    

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