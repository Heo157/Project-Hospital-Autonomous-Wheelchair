#!/usr/bin/env python3
"""
파일명: stm32_sensor_bridge.py
수정내용: STM32에서 오는 BTN(버튼) 값을 파싱하여 ROS 토픽으로 발행 기능 추가
"""

import re
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32  # [수정] Int32 추가

# 예: U=12.34,FSR=59,SEAT=0,BTN=2
# BTN 부분은 있을 수도 있고 없을 수도 있도록 (?: ... )? 로 처리
LINE_RE = re.compile(
    r'U\s*=\s*([-+]?\d+(?:\.\d+)?)\s*,\s*FSR\s*=\s*(\d+)\s*,\s*SEAT\s*=\s*(\d+)(?:,\s*BTN\s*=\s*(\d+))?'
)

class Stm32SensorBridge(Node):
    def __init__(self):
        super().__init__('stm32_sensor_bridge')

        # 파라미터
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('distance_topic', '/ultra_distance_cm')
        self.declare_parameter('seat_topic', '/seat_detected')
        self.declare_parameter('btn_topic', '/stm32/button') # [수정] 버튼 토픽 추가

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.distance_topic = self.get_parameter('distance_topic').value
        self.seat_topic = self.get_parameter('seat_topic').value
        self.btn_topic = self.get_parameter('btn_topic').value

        # 퍼블리셔
        self.pub_dist = self.create_publisher(Float32, self.distance_topic, 10)
        self.pub_seat = self.create_publisher(Bool, self.seat_topic, 10)
        self.pub_btn  = self.create_publisher(Int32, self.btn_topic, 10) # [수정] 버튼 퍼블리셔

        # 시리얼
        self.ser = None
        self.rx_buf = bytearray()
        self._open_serial()

        # 50Hz 폴링
        self.timer = self.create_timer(0.02, self._tick)

    def _open_serial(self):
        try:
            # timeout=0 : 논블로킹
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0)
            self.rx_buf.clear()
            self.get_logger().info(f"Serial opened: {self.port} @ {self.baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Serial open failed: {e}")

    def _process_line(self, line: str):
        line = line.strip()
        if not line:
            return

        m = LINE_RE.search(line)
        if not m:
            return

        dist = float(m.group(1))
        seat = int(m.group(3))
        seat_bool = (seat != 0)

        # [수정] 버튼 값 처리
        if m.group(4):
            btn_val = int(m.group(4))
            if btn_val != 0: # 버튼이 눌렸을 때만 발행 (혹은 항상 발행해도 무방)
                self.get_logger().info(f"🔘 Button Click Detected: {btn_val}")
                self.pub_btn.publish(Int32(data=btn_val))

        self.pub_dist.publish(Float32(data=dist))
        self.pub_seat.publish(Bool(data=seat_bool))

    def _tick(self):
        if self.ser is None:
            self._open_serial()
            return

        try:
            n = self.ser.in_waiting
            if n <= 0:
                return

            data = self.ser.read(n)
            if not data:
                return

            self.rx_buf.extend(data)

            # '\n' 단위로 프레임 분리
            while b'\n' in self.rx_buf:
                line_bytes, _, rest = self.rx_buf.partition(b'\n')
                self.rx_buf = bytearray(rest)

                line = line_bytes.decode(errors='ignore')
                self._process_line(line)

        except (OSError, serial.SerialException) as e:
            self.get_logger().warn(f"Serial error: {e} (reopen)")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

def main():
    rclpy.init()
    node = Stm32SensorBridge()
    try:
        rclpy.spin(node)
    finally:
        try:
            if node.ser:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()