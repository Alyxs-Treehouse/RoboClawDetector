import serial
import time
import struct

class LX16A_Tester:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        
        # LX-16A Commands
        self.SERVO_MOTOR_LOAD_OR_UNLOAD_WRITE = 31
        self.SERVO_MOTOR_LOAD_OR_UNLOAD_READ = 32
        self.SERVO_POS_READ = 28
        
    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1 # Short timeout for half-duplex RX
            )
            print(f"✓ 串口已连接: {self.port}")
            time.sleep(0.5)
            # Clear buffer
            self.serial.reset_input_buffer()
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口已断开")

    def _calculate_checksum(self, packet):
        """计算校验和：~ (ID + Length + Command + Param1 + ... Param N)"""
        chksum = sum(packet[2:])  # Sum ID through the end of params
        return (~chksum) & 0xFF

    def _send_command(self, servo_id, command, params=[]):
        """发送命令帧: 0x55 0x55 ID Length Command Params Checksum"""
        length = len(params) + 3 # Length = length(Params) + 1(Command) + 1(Length) + 1(Checksum)
        packet = [0x55, 0x55, servo_id, length, command] + params
        checksum = self._calculate_checksum(packet)
        packet.append(checksum)
        
        # Clear receive buffer before sending, critical for half-duplex
        self.serial.reset_input_buffer()
        self.serial.write(bytes(packet))
        
    def _receive_response(self, servo_id, expected_command, expected_length):
        """专门接收并解析返回帧"""
        # Read header 0x55 0x55
        header = self.serial.read(2)
        if len(header) != 2 or header[0] != 0x55 or header[1] != 0x55:
            return None
            
        ret_id = self.serial.read(1)
        if not ret_id or ret_id[0] != servo_id:
            return None
            
        length = self.serial.read(1)
        if not length:
            return None
            
        payload = self.serial.read(length[0] - 2) # cmd + params + checksum
        if len(payload) != length[0] - 2:
            return None
            
        cmd = payload[0]
        if cmd != expected_command:
            return None
            
        # Verify Checksum
        packet_for_chksum = [0x55, 0x55, ret_id[0], length[0]] + list(payload[:-1])
        calculated_chksum = self._calculate_checksum(packet_for_chksum)
        
        if calculated_chksum != payload[-1]:
            print(f"Checksum Error: Expected {calculated_chksum}, Got {payload[-1]}")
            return None
            
        # Return the params bytes
        return payload[1:-1]
        
    def unload_servo(self, servo_id):
        """命令 31: 掉电 / 卸载舵机"""
        print(f"尝试卸载舵机 ID: {servo_id}")
        self._send_command(servo_id, self.SERVO_MOTOR_LOAD_OR_UNLOAD_WRITE, [0])
        time.sleep(0.05)
        
    def load_servo(self, servo_id):
        """命令 31: 上电 / 加载舵机"""
        print(f"尝试加载舵机 ID: {servo_id}")
        self._send_command(servo_id, self.SERVO_MOTOR_LOAD_OR_UNLOAD_WRITE, [1])
        time.sleep(0.05)

    def read_position(self, servo_id):
        """命令 28: 读取当前位置"""
        self._send_command(servo_id, self.SERVO_POS_READ, [])
        # Receive: 55 55 ID 05 1C Prm1(Low) Prm2(High) Checksum
        # Length = 5, Cmd = 28 (0x1C), Total param return length = 2 bytes
        response = self._receive_response(servo_id, self.SERVO_POS_READ, 5)
        
        if response and len(response) == 2:
            # Little Endian unpacking of 2 bytes (unsigned short)
            position = struct.unpack('<h', response)[0]
            # Convert internal 0-1000 range to roughly 0-240 degrees representation or just return raw
            return position
        return None

if __name__ == "__main__":
    PORT = '/dev/tty.usbserial-210' # 你的实际串口
    tester = LX16A_Tester(PORT)
    
    if tester.connect():
        # 测试第一个关节舵机，ID = 0 （根据 servo_controller.py 中的定义）
        SERVO_ID = 0
        
        print("\n--- 第一步: 卸载电机 (脱力) ---")
        tester.unload_servo(SERVO_ID)
        print("舵机现在应该是松开的。请用手转动并观察控制台数据。")
        
        print("\n--- 第二步: 纯读取循环 ---")
        try:
            for i in range(30): # 测试读取30次，每次停顿0.5秒
                pos = tester.read_position(SERVO_ID)
                if pos is not None:
                    print(f"读取成功! 当前舵机内部数值: {pos}")
                else:
                    print("读取失败: 超时或无返回数据 (可能串口被占用或连线不稳)")
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
            
        print("\n--- 第三步: 恢复上电 (锁定) ---")
        tester.load_servo(SERVO_ID)
        print("舵机现已恢复上电。")
        
        tester.close()
