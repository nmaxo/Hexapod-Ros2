#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import SensorData  # Измененный импорт
import serial
import json
import threading
class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        
        # Инициализация последовательного порта
        self.ser = serial.Serial(
            port='/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0',
            baudrate=115200,
            timeout= 0.01
        )
        self.lock = threading.Lock()
        self.last_data = {str(i): 0.0 for i in range(1, 7)}  # Хранение последних значений
        self.running = True

        # Запуск фонового потока для чтения данных
        self.read_thread = threading.Thread(target=self._continuous_read)
        self.read_thread.start()
        # Publisher для данных ног
        self.publisher = self.create_publisher(SensorData, '/legs_data', 10)
        
        # Таймер для периодического чтения данных
        self.timer = self.create_timer(0.01, self.read_and_publish) 
        
        self.get_logger().info("SensorReader node started")


    def _continuous_read(self):
        """Фоновое чтение данных из серийного порта"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    self._process_line(line)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
                break
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

    def _process_line(self, line):
        """Парсинг и валидация данных"""
        try:
            data = json.loads(line)
            if not isinstance(data.get('legs'), dict):
                raise ValueError("Invalid legs format")
            
            with self.lock:
                for leg, value in data['legs'].items():
                    if leg in self.last_data:
                        try:
                            self.last_data[leg] = float(value)
                        except ValueError:
                            pass
        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid JSON: {line}")
        except ValueError as e:
            self.get_logger().warning(f"Data error: {e}")

    def read_and_publish(self):
        """Публикация текущих данных"""
        msg = SensorData()
        with self.lock:
            # Копируем данные для публикации
            msg.legs = [self.last_data[str(i+1)] for i in range(6)]
        
        self.publisher.publish(msg)
        # self.get_logger().info(f"Published: {msg.legs}")

    def stop(self):
        """Остановка фонового потока"""
        self.running = False
        self.read_thread.join()
def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
