"""
LewanSoul/Hiwonder LX-16A舵机控制器
使用串口协议: #XXXPYYYYTZZZ!
"""

import serial
import time
import math


class ServoController:
    def __init__(self, port='/dev/tty.usbserial-210', baudrate=115200):
        """初始化串口连接"""
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        
        # 舵机ID映射 (根据实际接线调整)
        self.servo_ids = {
            'joint_1': 0,  # 底座旋转
            'joint_2': 1,  # 肩部
            'joint_3': 2,  # 肘部
            'joint_4': 3,  # 腕部
            'joint_5': 4   # 末端旋转
        }
        
        # 舵机方向反转 (True = 反转)
        self.servo_reverse = {
            'joint_1': False,
            'joint_2': False,
            'joint_3': False,
            'joint_4': True,   # 反转
            'joint_5': False
        }
        
        # 舵机中位值 (1500 = 90度)
        self.servo_center = 1500
        
        # 舵机范围 (500-2500对应0-180度)
        self.servo_min = 500
        self.servo_max = 2500
        
        # 角度到舵机值的转换 (弧度 -> 舵机脉冲)
        # 假设: -2.356 rad (-135°) -> 500, +2.356 rad (+135°) -> 2500
        self.angle_range = 2.356  # ±135度
        
    def connect(self):
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            print(f"✓ 串口连接成功: {self.port}")
            time.sleep(0.1)  # 等待串口稳定
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口已关闭")
    
    def angle_to_servo(self, angle_rad):
        """将弧度角度转换为舵机脉冲值"""
        # angle_rad: -2.356 ~ +2.356
        # servo: 500 ~ 2500
        ratio = angle_rad / self.angle_range  # -1 ~ +1
        servo_value = self.servo_center + ratio * (self.servo_center - self.servo_min)
        return int(max(self.servo_min, min(self.servo_max, servo_value)))
    
    def send_servo_command(self, servo_id, position, duration_ms=1000):
        """发送舵机命令
        
        格式: #XXXPYYYYTZZZ!
        XXX: 舵机ID (000-255)
        YYYY: 位置 (0500-2500)
        ZZZ: 时间 (毫秒)
        """
        if not self.serial or not self.serial.is_open:
            return False
        
        # 构建命令字符串
        command = f"#{servo_id:03d}P{position:04d}T{duration_ms:04d}!\n"
        
        try:
            self.serial.write(command.encode())
            print(f"  → 舵机{servo_id}: {position} ({duration_ms}ms)")
            return True
        except Exception as e:
            print(f"✗ 发送命令失败: {e}")
            return False
    
    def move_joint(self, joint_name, angle_rad, duration_ms=1000):
        """移动单个关节"""
        if joint_name not in self.servo_ids:
            print(f"✗ 未知关节: {joint_name}")
            return False
        
        # 反转角度（如果需要）
        if self.servo_reverse.get(joint_name, False):
            angle_rad = -angle_rad
        
        servo_id = self.servo_ids[joint_name]
        servo_pos = self.angle_to_servo(angle_rad)
        
        print(f"  {joint_name}: {angle_rad:.3f} rad → servo {servo_pos}")
        
        return self.send_servo_command(servo_id, servo_pos, duration_ms)
    
    def move_all_joints(self, joint_angles, duration_ms=1000):
        """移动所有关节
        
        Args:
            joint_angles: 字典 {'joint_1': angle1, 'joint_2': angle2, ...}
            duration_ms: 移动时间（毫秒）
        """
        success = True
        for joint_name, angle in joint_angles.items():
            if joint_name in self.servo_ids:
                if not self.move_joint(joint_name, angle, duration_ms):
                    success = False
                time.sleep(0.002)  # 2ms延迟，足够串口发送
        return success
    
    def test_servos(self):
        """测试所有舵机"""
        print("\n测试舵机...")
        
        # 移动到中位
        print("移动到中位 (1500)...")
        for joint_name in self.servo_ids.keys():
            self.move_joint(joint_name, 0.0, 1000)
        time.sleep(1.5)
        
        # 测试每个关节
        for joint_name in self.servo_ids.keys():
            print(f"测试 {joint_name}...")
            self.move_joint(joint_name, 0.5, 500)  # +0.5 rad
            time.sleep(0.7)
            self.move_joint(joint_name, -0.5, 500)  # -0.5 rad
            time.sleep(0.7)
            self.move_joint(joint_name, 0.0, 500)  # 回中位
            time.sleep(0.7)
        
        print("测试完成")


if __name__ == "__main__":
    # 测试程序 - 修改为macOS串口格式
    controller = ServoController('/dev/tty.usbserial-210', 115200)
    
    if controller.connect():
        try:
            controller.test_servos()
        except KeyboardInterrupt:
            print("\n测试中断")
        finally:
            controller.disconnect()
