#!/usr/bin/env python3
"""
파일명: stm32_sensor_bridge.py
수정내용: STM32 데이터 포맷(값@값@값)에 맞춰 파싱 로직 전면 수정
"""

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32

class Stm32SensorBridge(Node):
    def __init__(self):
        super().__init__('stm32_sensor_bridge')

        # 파라미터
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('distance_topic', '/ultra_distance_cm')
        self.declare_parameter('seat_topic', '/seat_detected')
        self.declare_parameter('btn_topic', '/stm32/button')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.distance_topic = self.get_parameter('distance_topic').value
        self.seat_topic = self.get_parameter('seat_topic').value
        self.btn_topic = self.get_parameter('btn_topic').value

        # 퍼블리셔
        self.pub_dist = self.create_publisher(Float32, self.distance_topic, 10)
        self.pub_seat = self.create_publisher(Bool, self.seat_topic, 10)
        self.pub_btn  = self.create_publisher(Int32, self.btn_topic, 10)

        # 시리얼
        self.ser = None
        self.rx_buf = bytearray()
        self._open_serial()

        # 50Hz 폴링
        self.timer = self.create_timer(0.02, self._tick)

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0)
            self.rx_buf.clear()
            self.get_logger().info(f"Serial opened: {self.port} @ {self.baud}")
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Serial open failed: {e}")

    def _process_line(self, line: str):
        line = line.strip()
        if not line: return

        # [핵심 수정] 기존 정규식 제거 -> @ 기준으로 분리
        # 예상 포맷: "6.7@0@0" (거리@착석@버튼)
        parts = line.split('@')
        
        if len(parts) < 3:
            return  # 데이터 개수가 부족하면 무시

        try:
            # 1. 거리 (Float)
            dist = float(parts[0])
            
            # 2. 착석 (Int -> Bool)
            seat_val = int(parts[1])
            seat_bool = (seat_val != 0)
            
            # 3. 버튼 (Int)
            btn_val = int(parts[2])

            # 토픽 발행
            self.pub_dist.publish(Float32(data=dist))
            self.pub_seat.publish(Bool(data=seat_bool))
            
            # 버튼은 눌렸을 때만 로그 출력 (디버깅용)
            if btn_val != 0:
                self.get_logger().info(f"🔘 Button Click: {btn_val}")
            self.pub_btn.publish(Int32(data=btn_val))

        except ValueError:
            pass # 숫자가 아닌 값이 들어오면 무시

    def _tick(self):
        if self.ser is None:
            self._open_serial()
            return

        try:
            n = self.ser.in_waiting
            if n <= 0: return

            data = self.ser.read(n)
            if not data: return

            self.rx_buf.extend(data)

            while b'\n' in self.rx_buf:
                line_bytes, _, rest = self.rx_buf.partition(b'\n')
                self.rx_buf = bytearray(rest)
                
                # 디코딩 및 처리
                try:
                    line = line_bytes.decode('utf-8', errors='ignore')
                    self._process_line(line)
                except:
                    pass

        except (OSError, serial.SerialException) as e:
            self.get_logger().warn(f"Serial error: {e}")
            self.ser = None

def main():
    rclpy.init()
    node = Stm32SensorBridge()
    try: rclpy.spin(node)
    except: pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()