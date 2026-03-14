"""
机械臂WebSocket控制服务器

接收JSON格式的目标位置，使用IK求解并实时渲染机械臂运动
"""

import asyncio
import websockets
import json
import threading
import time
import numpy as np
import queue
from robot_renderer import RobotRenderer
from robot_ik import RobotIK
from servo_controller import ServoController


class RobotWebSocketServer:
    def __init__(self, urdf_file, host='0.0.0.0', port=8765, enable_servo=False, servo_port='/dev/tty.usbserial-210', servo_baudrate=115200):
        self.host = host
        self.port = port
        self.renderer = RobotRenderer(urdf_file)
        self.ik_solver = RobotIK(urdf_file)
        
        # 舵机控制器
        self.enable_servo = enable_servo
        self.servo_controller = None
        if enable_servo:
            self.servo_controller = ServoController(servo_port, servo_baudrate)
            if self.servo_controller.connect():
                print("✓ 舵机控制器已启用")
            else:
                print("✗ 舵机控制器连接失败，仅使用虚拟渲染")
                self.servo_controller = None
        
        # 当前状态
        self.current_angles = {
            'joint_1': 0.0,
            'joint_2': 0.0,
            'joint_3': 0.0,
            'joint_4': 0.0,
            'joint_5': 0.0
        }
        
        # 运动控制
        self.target_angles = None
        self.is_moving = False
        self.move_duration = 1.5  # 默认移动时间（秒）
        self.move_start_time = 0
        self.move_start_angles = None
        
        # 连接的客户端
        self.clients = set()
        
        # 统计信息
        self.total_commands = 0
        self.successful_moves = 0
        self.failed_moves = 0
        
        # 命令队列（用于线程间通信）
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        
    async def handle_client(self, websocket):
        """处理客户端连接"""
        client_addr = websocket.remote_address
        self.clients.add(websocket)
        print(f"✓ 客户端连接: {client_addr}")
        
        try:
            async for message in websocket:
                await self.process_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            print(f"✗ 客户端断开: {client_addr}")
        finally:
            self.clients.remove(websocket)
    
    async def process_message(self, websocket, message):
        """处理接收到的消息"""
        try:
            data = json.loads(message)
            command = data.get('command', 'move')
            
            if command == 'move':
                await self.handle_move_command(websocket, data)
            elif command == 'gripper':
                await self.handle_gripper_command(websocket, data)
            elif command == 'get_status':
                await self.handle_status_command(websocket)
            elif command == 'get_position':
                await self.handle_get_position_command(websocket)
            elif command == 'set_speed':
                await self.handle_set_speed_command(websocket, data)
            elif command == 'set_joints':
                await self.handle_set_joints_command(websocket, data)
            else:
                await self.send_error(websocket, f"未知命令: {command}")
                
        except json.JSONDecodeError:
            await self.send_error(websocket, "无效的JSON格式")
        except Exception as e:
            await self.send_error(websocket, f"处理错误: {str(e)}")
    
    async def handle_move_command(self, websocket, data):
        """处理移动命令"""
        self.total_commands += 1
        
        # 解析目标位置（工作空间坐标）
        if 'position' in data:
            pos = data['position']
            target_workspace = [pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)]
        elif 'x' in data and 'y' in data and 'z' in data:
            target_workspace = [data['x'], data['y'], data['z']]
        else:
            await self.send_error(websocket, "缺少位置信息 (x, y, z)")
            return
        
        # 解析目标roll角度（可选）
        target_roll = data.get('roll', None)
        
        # 转换为世界坐标
        target_world = self.ik_solver.workspace_to_world(target_workspace)
        
        # 获取移动时间
        duration = data.get('duration', self.move_duration)
        
        print(f"\n📡 收到移动指令 (工作空间): [{target_workspace[0]:.3f}, {target_workspace[1]:.3f}, {target_workspace[2]:.3f}]")
        if target_roll is not None:
            print(f"   Roll角度: {target_roll:.3f} rad ({np.degrees(target_roll):.1f}°)")
        
        # 求解IK（使用世界坐标，传入当前角度保持连续性）
        solution = self.ik_solver.solve_ik_multiple_attempts(
            target_world, 
            num_attempts=10, 
            target_roll=target_roll,
            previous_angles=self.current_angles
        )
        
        if solution is None:
            self.failed_moves += 1
            print(f"❌ IK求解失败，目标位置不可达")
            await self.send_response(websocket, {
                'status': 'error',
                'message': '目标位置不可达',
                'target': target_workspace
            })
            return
        
        # 验证解（世界坐标）
        achieved_world = self.ik_solver.forward_kinematics(solution)
        achieved_workspace = self.ik_solver.world_to_workspace(achieved_world)
        error = np.linalg.norm(achieved_world - np.array(target_world))
        
        if error > 0.05:  # 50mm error threshold
            self.failed_moves += 1
            print(f"❌ 目标位置不可达 (解算误差: {error*1000:.2f} mm > 50 mm)")
            await self.send_response(websocket, {
                'status': 'error',
                'message': f'目标位置不可达 (误差 {error*1000:.2f}mm)',
                'target': target_workspace
            })
            return
            
        print(f"✓ IK求解成功，误差: {error*1000:.2f} mm")
        
        # 立即开始移动（不等待完成）
        self.start_move(solution, duration)
        
        # 立即返回响应（工作空间坐标）
        self.successful_moves += 1
        print(f"✓ 开始移动")
        await self.send_response(websocket, {
            'status': 'success',
            'message': '开始移动',
            'target': target_workspace,
            'achieved': achieved_workspace.tolist(),
            'error_mm': float(error * 1000),
            'joint_angles': {k: float(v) for k, v in solution.items()}
        })
        
    async def handle_set_joints_command(self, websocket, data):
        """处理直接设置关节角度命令 (来自虚拟示教器)"""
        if 'angles' not in data:
            await self.send_error(websocket, "缺少 angles 参数")
            return
            
        angles = data['angles']
        duration = data.get('duration', 0.1) # 默认极短时间响应 UI 拖动
        
        # 将接收到的角度存入 dict
        target_angles = {
            'joint_1': float(angles.get('joint_1', self.current_angles['joint_1'])),
            'joint_2': float(angles.get('joint_2', self.current_angles['joint_2'])),
            'joint_3': float(angles.get('joint_3', self.current_angles['joint_3'])),
            'joint_4': float(angles.get('joint_4', self.current_angles['joint_4'])),
            'joint_5': float(angles.get('joint_5', self.current_angles['joint_5']))
        }
        
        # 立即开始移动
        self.start_move(target_angles, duration)
        
        # 顺便获取新的当前世界坐标作为回传验证（可选）
        current_pos_world = self.ik_solver.forward_kinematics(target_angles)
        current_pos_workspace = self.ik_solver.world_to_workspace(current_pos_world)
        
        await self.send_response(websocket, {
            'status': 'success',
            'message': '开始关节运动',
            'achieved': current_pos_workspace.tolist()
        })
    
    async def handle_gripper_command(self, websocket, data):
        """处理夹爪命令"""
        if 'position' not in data:
            await self.send_error(websocket, "缺少position参数")
            return
        
        position = int(data['position'])
        
        # 发送到舵机5（夹爪）
        if self.servo_controller:
            self.servo_controller.send_servo_command(5, position, 500)
            print(f"🤏 夹爪: {position}")
        
        await self.send_response(websocket, {
            'status': 'success',
            'message': '夹爪命令已发送',
            'position': position
        })
    
    async def handle_status_command(self, websocket):
        """处理状态查询命令"""
        current_pos_world = self.ik_solver.forward_kinematics(self.current_angles)
        current_pos_workspace = self.ik_solver.world_to_workspace(current_pos_world)
        
        status = {
            'status': 'ok',
            'is_moving': self.is_moving,
            'current_position': {
                'x': float(current_pos_workspace[0]),
                'y': float(current_pos_workspace[1]),
                'z': float(current_pos_workspace[2])
            },
            'current_angles': {k: float(v) for k, v in self.current_angles.items()},
            'statistics': {
                'total_commands': self.total_commands,
                'successful_moves': self.successful_moves,
                'failed_moves': self.failed_moves
            },
            'connected_clients': len(self.clients)
        }
        
        await self.send_response(websocket, status)
    
    async def handle_get_position_command(self, websocket):
        """处理获取当前位置命令"""
        current_pos_world = self.ik_solver.forward_kinematics(self.current_angles)
        current_pos_workspace = self.ik_solver.world_to_workspace(current_pos_world)
        
        response = {
            'status': 'ok',
            'position': {
                'x': float(current_pos_workspace[0]),
                'y': float(current_pos_workspace[1]),
                'z': float(current_pos_workspace[2])
            }
        }
        
        await self.send_response(websocket, response)
    
    async def handle_set_speed_command(self, websocket, data):
        """处理设置速度命令"""
        if 'duration' in data:
            self.move_duration = float(data['duration'])
            await self.send_response(websocket, {
                'status': 'ok',
                'message': f'移动时间设置为 {self.move_duration} 秒'
            })
        else:
            await self.send_error(websocket, "缺少 duration 参数")
    
    async def send_response(self, websocket, data):
        """发送响应"""
        await websocket.send(json.dumps(data))
    
    async def send_error(self, websocket, message):
        """发送错误响应"""
        await websocket.send(json.dumps({
            'status': 'error',
            'message': message
        }))
    
    def start_move(self, target_angles, duration):
        """开始移动到目标角度"""
        self.is_moving = True
        self.move_start_angles = self.current_angles.copy()
        self.target_angles = target_angles
        self.move_duration = duration
        self.move_start_time = time.time()
        
        # 发送命令到真实舵机
        if self.servo_controller:
            print("发送到舵机:")
            duration_ms = int(duration * 1000)
            self.servo_controller.move_all_joints(target_angles, duration_ms)
    
    def update_movement(self):
        """更新运动状态（在主线程的渲染循环中调用）"""
        if not self.is_moving:
            return
        
        elapsed = time.time() - self.move_start_time
        
        if elapsed >= self.move_duration:
            # 移动完成
            self.current_angles = self.target_angles.copy()
            self.is_moving = False
        else:
            # 插值计算当前角度
            t = elapsed / self.move_duration
            # 平滑插值（ease-in-out）
            smooth_t = t * t * (3 - 2 * t)
            
            for joint in ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']:
                self.current_angles[joint] = (
                    self.move_start_angles[joint] * (1 - smooth_t) + 
                    self.target_angles[joint] * smooth_t
                )
        
        # 更新渲染器
        self.renderer.update_joint_angles(self.current_angles)
    
    def run_websocket_server(self):
        """运行WebSocket服务器"""
        async def start_server():
            print("=" * 60)
            print("机械臂WebSocket控制服务器")
            print("=" * 60)
            print(f"服务器地址: ws://{self.host}:{self.port}")
            if self.servo_controller:
                print(f"舵机控制: 已启用 ({self.servo_controller.port})")
            else:
                print("舵机控制: 未启用 (仅虚拟渲染)")
            print("=" * 60)
            print("\n支持的命令:")
            print("1. 移动到目标位置:")
            print('   {"command": "move", "x": 0.1, "y": 0.1, "z": 0.3}')
            print('   或 {"command": "move", "position": {"x": 0.1, "y": 0.1, "z": 0.3}, "duration": 2.0}')
            print("\n2. 查询状态:")
            print('   {"command": "get_status"}')
            print("\n3. 获取当前位置:")
            print('   {"command": "get_position"}')
            print("\n4. 设置移动速度:")
            print('   {"command": "set_speed", "duration": 1.5}')
            print("\n5. 直接设置关节角度 (虚拟示教器):")
            print('   {"command": "set_joints", "angles": {"joint_1": 0.5, "joint_2": -0.5...}, "duration": 0.1}')
            print("\n" + "=" * 60)
            print("等待客户端连接...\n")
            
            async with websockets.serve(self.handle_client, self.host, self.port):
                await asyncio.Future()  # 永远运行
        
        asyncio.run(start_server())
    
    def run(self):
        """启动服务器和渲染器"""
        # 在单独的线程运行WebSocket服务器
        server_thread = threading.Thread(target=self.run_websocket_server, daemon=True)
        server_thread.start()
        
        # 主线程运行渲染循环
        time.sleep(1)  # 等待服务器启动
        
        import pygame
        clock = pygame.time.Clock()
        running = True
        
        while running:
            # 处理事件
            running = self.renderer.handle_events()
            
            # 更新运动状态
            self.update_movement()
            
            # 渲染
            self.renderer.render()
            
            # 控制帧率
            clock.tick(60)
        
        pygame.quit()
        
        # 关闭舵机控制器
        if self.servo_controller:
            self.servo_controller.disconnect()


if __name__ == "__main__":
    import sys
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='机械臂WebSocket控制服务器')
    parser.add_argument('--host', default='0.0.0.0', help='服务器地址')
    parser.add_argument('--port', type=int, default=8765, help='服务器端口')
    parser.add_argument('--servo', action='store_true', help='启用舵机控制')
    parser.add_argument('--servo-port', default='/dev/tty.usbserial-210', help='舵机串口')
    parser.add_argument('--servo-baudrate', type=int, default=115200, help='舵机波特率')
    
    args = parser.parse_args()
    
    # 创建并运行服务器
    server = RobotWebSocketServer(
        "robot_template.urdf",
        host=args.host,
        port=args.port,
        enable_servo=args.servo,
        servo_port=args.servo_port,
        servo_baudrate=args.servo_baudrate
    )
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("\n服务器关闭")
