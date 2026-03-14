"""
机械臂IK演示
展示如何使用IK控制机械臂移动到目标位置
"""

import numpy as np
import time
import threading
from robot_renderer import RobotRenderer
from robot_ik import RobotIK


class RobotIKDemo:
    def __init__(self, urdf_file):
        self.renderer = RobotRenderer(urdf_file)
        self.ik_solver = RobotIK(urdf_file)
        self.current_angles = {
            'joint_1': 0.0,
            'joint_2': 0.0,
            'joint_3': 0.0,
            'joint_4': 0.0,
            'joint_5': 0.0
        }
        self.target_angles = self.current_angles.copy()
        self.is_moving = False
        
    def move_to_position(self, target_position, duration=2.0):
        """平滑移动到目标位置
        
        Args:
            target_position: 目标位置 [x, y, z]
            duration: 移动时间（秒）
        """
        print(f"\n目标位置: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
        
        # 求解IK
        solution = self.ik_solver.solve_ik_multiple_attempts(target_position, num_attempts=10)
        
        if solution is None:
            print("❌ IK求解失败，目标位置不可达")
            return False
        
        # 验证解
        achieved = self.ik_solver.forward_kinematics(solution)
        error = np.linalg.norm(achieved - np.array(target_position))
        print(f"✓ IK求解成功，误差: {error*1000:.2f} mm")
        
        # 平滑插值移动
        start_angles = self.current_angles.copy()
        self.target_angles = solution
        
        steps = int(duration * 50)  # 50Hz更新
        for i in range(steps + 1):
            t = i / steps
            # 使用平滑插值（ease-in-out）
            smooth_t = t * t * (3 - 2 * t)
            
            interpolated = {}
            for joint in ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']:
                interpolated[joint] = (
                    start_angles[joint] * (1 - smooth_t) + 
                    self.target_angles[joint] * smooth_t
                )
            
            self.current_angles = interpolated
            self.renderer.update_joint_angles(self.current_angles)
            time.sleep(0.02)
        
        print(f"✓ 移动完成")
        return True
    
    def demo_sequence(self):
        """演示序列：移动到多个目标点"""
        print("=" * 60)
        print("机械臂IK演示")
        print("=" * 60)
        print("控制:")
        print("  - 左键拖动: 旋转视角")
        print("  - 滚轮: 缩放")
        print("  - ESC/Q: 退出")
        print("=" * 60)
        
        # 等待一下让用户看到初始状态
        time.sleep(2)
        
        # 定义一系列目标点
        targets = [
            [0.15, 0.0, 0.25],   # 前方
            [0.0, 0.15, 0.25],   # 左侧
            [-0.1, 0.0, 0.30],   # 后上方
            [0.0, -0.15, 0.20],  # 右下方
            [0.1, 0.1, 0.35],    # 前左上方
            [0.0, 0.0, 0.30],    # 中心上方（回到初始区域）
        ]
        
        print("\n开始演示序列...")
        
        for i, target in enumerate(targets, 1):
            print(f"\n--- 目标点 {i}/{len(targets)} ---")
            success = self.move_to_position(target, duration=2.0)
            if success:
                time.sleep(1)  # 在目标点停留1秒
            else:
                print("跳过此目标点")
                time.sleep(0.5)
        
        print("\n" + "=" * 60)
        print("演示完成！")
        print("=" * 60)


def demo_interactive():
    """交互式演示：手动输入目标位置"""
    demo = RobotIKDemo("robot_template.urdf")
    
    # 在后台线程运行演示序列
    def run_demo():
        time.sleep(1)  # 等待窗口初始化
        demo.demo_sequence()
    
    demo_thread = threading.Thread(target=run_demo, daemon=True)
    demo_thread.start()
    
    # 运行渲染循环
    demo.renderer.run()


def demo_network_simulation():
    """模拟从网络接收目标位置"""
    demo = RobotIKDemo("robot_template.urdf")
    
    print("=" * 60)
    print("网络控制模拟")
    print("=" * 60)
    print("模拟从网络接收目标坐标...")
    print("=" * 60)
    
    def network_receiver():
        """模拟网络接收线程"""
        time.sleep(2)
        
        # 模拟接收到的目标位置序列
        while True:
            # 在实际应用中，这里会从网络接收数据
            # target = receive_from_network()
            
            # 模拟：生成随机目标位置
            target = [
                np.random.uniform(-0.1, 0.2),
                np.random.uniform(-0.15, 0.15),
                np.random.uniform(0.15, 0.35)
            ]
            
            print(f"\n📡 收到网络指令: 移动到 {target}")
            demo.move_to_position(target, duration=1.5)
            time.sleep(2)  # 等待下一个指令
    
    network_thread = threading.Thread(target=network_receiver, daemon=True)
    network_thread.start()
    
    demo.renderer.run()


if __name__ == "__main__":
    import sys
    
    print("\n选择演示模式:")
    print("1. 自动演示序列（推荐）")
    print("2. 网络控制模拟")
    
    choice = input("\n请输入选择 (1-2，默认1): ").strip()
    
    if choice == '2':
        demo_network_simulation()
    else:
        demo_interactive()
