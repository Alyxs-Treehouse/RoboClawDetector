"""
Plan B: 机械臂预定义动作模块

直接通过串口控制舵机，不经过 WebSocket。
提供简单 API 供 OpenClaw 调用。

重要: macOS 上关闭串口会导致 USB-串口芯片锁死，
因此串口在 __init__ 时打开，全程保持连接不关闭。

用法:
    from arm_actions import ArmActions
    arm = ArmActions()  # 打开串口，保持连接
    arm.center()
    arm.nod()
    arm.nod()        # 可以反复调用，不需要重新连接
    arm.disconnect()  # 只在彻底结束时调用

CLI 测试:
    python arm_actions.py center
    python arm_actions.py nod
    python arm_actions.py center nod  # 可以连续执行多个动作
"""

import serial
import time
import sys


class ArmActions:
    """机械臂预定义动作控制器（持久串口连接）"""

    def __init__(self, port='/dev/tty.usbserial-210', baudrate=115200):
        self.port = port
        self.baudrate = baudrate

        # 舵机 ID 映射
        self.JOINT_1 = 0   # 底座旋转 (摇头)
        self.JOINT_2 = 1   # 肩部
        self.JOINT_3 = 2   # 肘部
        self.JOINT_4 = 3   # 腕部
        self.JOINT_5 = 4   # 末端旋转
        self.GRIPPER = 5   # 夹爪

        # 舵机方向反转
        self.reverse = {3: True}  # joint_4 反转

        # 舵机中位和范围
        self.CENTER = 1500
        self.MIN = 500
        self.MAX = 2500
        self.ANGLE_RANGE = 2.356  # ±135°

        # 打开串口并保持连接（禁用 DTR/RTS 防止 ESP32 复位）
        print(f"🔌 连接串口 {port}...")
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 0.5
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(5)  # 等待 ESP32 启动完成
        self.ser.reset_input_buffer()
        print("✓ 串口已连接（持久连接模式）")

    def disconnect(self):
        """断开串口（仅在彻底结束时调用）"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 串口已断开")

    # ─── 底层控制 ──────────────────────────────────────

    def _angle_to_servo(self, angle_rad):
        """弧度 → 舵机脉冲值 (500-2500)"""
        ratio = angle_rad / self.ANGLE_RANGE
        value = self.CENTER + ratio * (self.CENTER - self.MIN)
        return int(max(self.MIN, min(self.MAX, value)))

    def _send_cmd(self, servo_id, angle_rad, duration_ms):
        """发送单个舵机命令"""
        angle = -angle_rad if servo_id in self.reverse else angle_rad
        servo_pos = self._angle_to_servo(angle)
        cmd = f"#{servo_id:03d}P{servo_pos:04d}T{duration_ms:04d}!\n"
        self.ser.write(cmd.encode())
        self.ser.flush()
        print(f"  舵机{servo_id}: {servo_pos} ({duration_ms}ms)")
        time.sleep(0.002)

    def _send_raw(self, servo_id, position, duration_ms):
        """发送单个舵机原始脉冲值命令"""
        cmd = f"#{servo_id:03d}P{position:04d}T{duration_ms:04d}!\n"
        self.ser.write(cmd.encode())
        self.ser.flush()
        print(f"  舵机{servo_id}: {position} ({duration_ms}ms)")
        time.sleep(0.002)

    def _run_sequence(self, keyframes):
        """执行一个动作序列

        Args:
            keyframes: list of (angles_dict, duration_ms, wait_sec)
                angles_dict: {servo_id: angle_rad, ...}
                duration_ms: 舵机运动时间
                wait_sec: 发完本帧后等待多久再发下一帧
        """
        for angles, duration_ms, wait_sec in keyframes:
            for servo_id, angle_rad in angles.items():
                self._send_cmd(servo_id, angle_rad, duration_ms)
            time.sleep(wait_sec)

    # ─── 预定义动作 ──────────────────────────────────────

    ALL_CENTER = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}

    def center(self):
        """动作 1: 置中 — 所有关节回到中位"""
        print("🤖 动作: 置中")
        self._run_sequence([
            (self.ALL_CENTER, 1000, 1.2),
        ])
        print("✓ 置中完成")

    def nod(self, times=3):
        """动作 3: 点头 — joint_4 (腕部，从上往下第一个) 前后摆动"""
        print(f"🤖 动作: 点头 (×{times})")
        keyframes = []
        for _ in range(times):
            # 低头 (腕部前倾)
            keyframes.append(({self.JOINT_4: 0.4}, 500, 0.6))
            # 回中
            keyframes.append(({self.JOINT_4: 0.0}, 500, 0.6))
        self._run_sequence(keyframes)
        print("✓ 点头完成")

    def shake_head(self, times=3):
        """动作 4: 摇头 — joint_1 (底座) 左右摆动"""
        print(f"🤖 动作: 摇头 (×{times})")
        keyframes = []
        for _ in range(times):
            # 向左
            keyframes.append(({self.JOINT_1: 0.4}, 500, 0.6))
            # 向右
            keyframes.append(({self.JOINT_1: -0.4}, 500, 0.6))
        # 回中
        keyframes.append(({self.JOINT_1: 0.0}, 500, 0.6))
        self._run_sequence(keyframes)
        print("✓ 摇头完成")

    def shrug(self):
        """动作 5: 摊手 — 爷子张平，夹爪张开"""
        print("🤖 动作: 摊手")
        # 手臂稍微抬起 + 腕部旋转到水平 + 夹爪张开
        self._run_sequence([
            # 抬起手臂，腕部旋转
            ({self.JOINT_2: -0.3, self.JOINT_3: 0.3, self.JOINT_5: 1.2}, 800, 1.0),
        ])
        # 张开夹爪 (直接发原始脉冲值，张开位置)
        self._send_raw(self.GRIPPER, 1200, 500)
        time.sleep(2.0)  # 保持摊手姿势
        # 收回
        self._send_raw(self.GRIPPER, 1500, 500)
        time.sleep(0.5)
        self._run_sequence([
            (self.ALL_CENTER, 800, 1.0),
        ])
        print("✓ 摊手完成")

    def grab_bag(self):
        """动作 2: 抓袋子 — 伸到前面抓取"""
        print("🤖 动作: 抓袋子")
        # 1. 张开夹爪
        self._send_raw(self.GRIPPER, 1200, 500)
        time.sleep(0.6)
        # 2. 伸到前方抓取位置（手臂前倾+低头）
        self._run_sequence([
            ({self.JOINT_2: 0.8, self.JOINT_3: 0.5, self.JOINT_4: 0.4}, 1000, 1.2),
        ])
        # 3. 闭合夹爪（抓住）
        self._send_raw(self.GRIPPER, 1500, 500)
        time.sleep(0.8)
        # 4. 抬起（回到中位）
        self._run_sequence([
            (self.ALL_CENTER, 1000, 1.2),
        ])
        print("✓ 抓袋子完成")

    def point_forward(self):
        """动作 6: 指向前侧面 — 爪子伸到底座平齐高度"""
        print("🤖 动作: 指向前方")
        self._run_sequence([
            # 底座偏转侧面 + 肩部前倾 + 肘部伸展 + 腕部补偿
            ({self.JOINT_1: 0.5, self.JOINT_2: 1.0, self.JOINT_3: 0.5, self.JOINT_4: 0.3, self.JOINT_5: 0.0}, 1200, 2.0),
        ])
        # 回中
        self._run_sequence([
            (self.ALL_CENTER, 1000, 1.2),
        ])
        print("✓ 指向前方完成")

    def point_down(self):
        """动作 7: 由上而下指人 — 手臂从竖直向上缓缓下压，末端指向目标"""
        print("🤖 动作: 由上而下指人")
        self._run_sequence([
            # Phase 1: 手臂竖直朝上（起始姿态）
            (self.ALL_CENTER, 800, 1.0),
            # Phase 2: 肩部缓缓前倾，同时肘部展开，腕部保持指向前方
            ({self.JOINT_2: 0.5, self.JOINT_3: 0.3, self.JOINT_4: -0.2}, 800, 0.8),
            # Phase 3: 继续下压，末端指向人体高度
            ({self.JOINT_2: 1.0, self.JOINT_3: 0.6, self.JOINT_4: 0.4}, 700, 1.5),
        ])
        # 保持指向姿态 1 秒后收回
        self._run_sequence([
            (self.ALL_CENTER, 1000, 1.2),
        ])
        print("✓ 由上而下指人完成")

    def shoot(self):
        """Recorded motion: shoot"""
        print("🤖 动作: shoot")
        keyframes = [
            ({0: 1508, 1: 1500, 2: 1500, 3: 1490, 4: 1514}, 800, 0.82),
            ({0: 1508, 1: 1500, 2: 1500, 3: 1490, 4: 1514}, 1805, 1.825),
            ({0: 1500, 1: 1510, 2: 1500, 3: 1490, 4: 1514}, 1805, 1.825),
            ({0: 1500, 1: 1500, 2: 1500, 3: 1490, 4: 1514}, 1804, 1.824),
        ]
        # Use raw positions directly
        for positions, duration_ms, wait_sec in keyframes:
            for sid, pos in positions.items():
                self._send_raw(sid, pos, duration_ms)
            time.sleep(wait_sec)
        self._run_sequence([(self.ALL_CENTER, 800, 1.0)])
        print("✓ shoot 完成")

    def kick(self):
        """动作 8: 踢一下 — 手臂前伸，用夹头进行快速跑/弹的动作"""
        print("🤖 动作: 踢一下")
        # 夹爪闭合，作为"脚"
        self._send_raw(self.GRIPPER, 1500, 500)
        time.sleep(0.5)
        
        self._run_sequence([
            # Phase 1: 蓄力（手臂压低，腕部不动）
            ({self.JOINT_2: 1.1, self.JOINT_3: 0.8, self.JOINT_4: 0.0}, 800, 0.8),
            # Phase 2: 腕部先反向蓄力
            ({self.JOINT_2: 1.0, self.JOINT_3: 0.7, self.JOINT_4: 0.5}, 400, 0.5),
            # Phase 3: 踢出（腕部快速反向甩）
            ({self.JOINT_2: 0.7, self.JOINT_3: 0.4, self.JOINT_4: -1.0}, 200, 0.8),
            # Phase 3: 收回中位
            (self.ALL_CENTER, 800, 1.0)
        ])
        print("✓ 踢一下完成")

    def dance(self):
        """动作 9: 跳舞 — 小幅度扭一扭"""
        print("🤖 动作: 跳舞")
        keyframes = []
        for _ in range(4):
            # 左扭（底座左转+肩微倾+腕微摆）
            keyframes.append(({self.JOINT_1: 0.3, self.JOINT_2: -0.15, self.JOINT_4: 0.2}, 350, 0.4))
            # 右扭
            keyframes.append(({self.JOINT_1: -0.3, self.JOINT_2: 0.15, self.JOINT_4: -0.2}, 350, 0.4))
        # 回中
        keyframes.append((self.ALL_CENTER, 500, 0.6))
        self._run_sequence(keyframes)
        print("✓ 跳舞完成")
