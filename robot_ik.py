import numpy as np
from scipy.optimize import minimize
import xml.etree.ElementTree as ET


class RobotIK:
    """机械臂逆运动学求解器
    
    只控制关节0/1/2/3（joint_1到joint_4），关节4（joint_5）保持为0
    """
    
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.links, self.joints = self.parse_urdf(urdf_file)
        
        # 关节限制（弧度）
        self.joint_limits = {
            'joint_1': (-2.356, 2.356),  # ±135度
            'joint_2': (-2.356, 2.356),
            'joint_3': (-2.356, 2.356),
            'joint_4': (-2.356, 2.356),
        }
        
        # 计算工作空间原点（所有关节为0时的末端位置）
        self.workspace_origin = self.forward_kinematics([0, 0, 0, 0])
    
    def workspace_to_world(self, workspace_pos):
        """将工作空间坐标转换为世界坐标"""
        return np.array(workspace_pos) + self.workspace_origin
    
    def world_to_workspace(self, world_pos):
        """将世界坐标转换为工作空间坐标"""
        return np.array(world_pos) - self.workspace_origin
        
    def parse_urdf(self, filename):
        """解析URDF文件"""
        tree = ET.parse(filename)
        root = tree.getroot()
        
        links = {}
        joints = {}
        
        # 解析links
        for link in root.findall('link'):
            name = link.get('name')
            visual = link.find('visual')
            if visual is not None:
                geometry = visual.find('geometry')
                origin = visual.find('origin')
                
                geom_info = {}
                if geometry.find('box') is not None:
                    box = geometry.find('box')
                    size = [float(x) for x in box.get('size').split()]
                    geom_info = {'type': 'box', 'size': size}
                elif geometry.find('cylinder') is not None:
                    cylinder = geometry.find('cylinder')
                    geom_info = {
                        'type': 'cylinder',
                        'radius': float(cylinder.get('radius')),
                        'length': float(cylinder.get('length'))
                    }
                
                xyz = [0, 0, 0]
                rpy = [0, 0, 0]
                if origin is not None:
                    if origin.get('xyz'):
                        xyz = [float(x) for x in origin.get('xyz').split()]
                    if origin.get('rpy'):
                        rpy = [float(x) for x in origin.get('rpy').split()]
                
                links[name] = {
                    'geometry': geom_info,
                    'origin': xyz,
                    'rpy': rpy
                }
        
        # 解析joints
        for joint in root.findall('joint'):
            name = joint.get('name')
            joint_type = joint.get('type')
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            origin = joint.find('origin')
            axis = joint.find('axis')
            
            xyz = [0, 0, 0]
            rpy = [0, 0, 0]
            if origin is not None:
                if origin.get('xyz'):
                    xyz = [float(x) for x in origin.get('xyz').split()]
                if origin.get('rpy'):
                    rpy = [float(x) for x in origin.get('rpy').split()]
            
            axis_vec = [0, 0, 1]
            if axis is not None and axis.get('xyz'):
                axis_vec = [float(x) for x in axis.get('xyz').split()]
            
            joints[name] = {
                'type': joint_type,
                'parent': parent,
                'child': child,
                'origin': xyz,
                'rpy': rpy,
                'axis': axis_vec
            }
        
        return links, joints
    
    def rotation_matrix(self, axis, angle):
        """根据轴和角度创建旋转矩阵"""
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)
        a = np.cos(angle / 2.0)
        b, c, d = -axis * np.sin(angle / 2.0)
        
        return np.array([
            [a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
            [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
            [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]
        ])
    
    def rpy_to_rotation_matrix(self, rpy):
        """将roll-pitch-yaw转换为旋转矩阵"""
        roll, pitch, yaw = rpy
        
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        return R_z @ R_y @ R_x
    
    def forward_kinematics(self, joint_angles):
        """正运动学：计算末端执行器位置
        
        Args:
            joint_angles: 字典或numpy数组，关节角度（弧度）
                         如果是数组，顺序为 [joint_1, joint_2, joint_3, joint_4]
        
        Returns:
            末端执行器的3D位置 [x, y, z]
        """
        if isinstance(joint_angles, (list, np.ndarray)):
            angles = {
                'joint_1': float(joint_angles[0]),
                'joint_2': float(joint_angles[1]),
                'joint_3': float(joint_angles[2]),
                'joint_4': float(joint_angles[3]),
                'joint_5': 0.0  # 固定为0
            }
        else:
            angles = joint_angles.copy()
            if 'joint_5' not in angles:
                angles['joint_5'] = 0.0
        
        # 计算变换
        transforms = {}
        transforms['base_link'] = {
            'position': np.array([0, 0, 0]),
            'rotation': np.eye(3)
        }
        
        # 按照关节链计算变换
        for joint_name, joint_info in self.joints.items():
            parent = joint_info['parent']
            child = joint_info['child']
            
            if parent in transforms:
                parent_pos = transforms[parent]['position']
                parent_rot = transforms[parent]['rotation']
                
                joint_offset = np.array(joint_info['origin'])
                joint_rpy = joint_info['rpy']
                joint_rot = self.rpy_to_rotation_matrix(joint_rpy)
                
                if joint_info['type'] == 'revolute':
                    angle = angles.get(joint_name, 0.0)
                    axis = joint_info['axis']
                    angle_rot = self.rotation_matrix(axis, angle)
                    joint_rot = joint_rot @ angle_rot
                
                child_pos = parent_pos + parent_rot @ joint_offset
                child_rot = parent_rot @ joint_rot
                
                transforms[child] = {
                    'position': child_pos,
                    'rotation': child_rot
                }
        
        # 返回最后一个link的位置（末端执行器）
        end_effector_link = 'link_5'
        if end_effector_link in transforms:
            pos = transforms[end_effector_link]['position']
            # 加上link_5的origin偏移
            if end_effector_link in self.links:
                origin_offset = np.array(self.links[end_effector_link]['origin'])
                rot = transforms[end_effector_link]['rotation']
                pos = pos + rot @ origin_offset
            return pos
        
        return np.array([0, 0, 0])
    
    def inverse_kinematics(self, target_position, initial_guess=None, method='SLSQP', target_roll=None):
        """逆运动学：计算到达目标位置的关节角度
        
        Args:
            target_position: 目标位置 [x, y, z]
            initial_guess: 初始猜测的关节角度 [j1, j2, j3, j4]，如果为None则使用全0
            method: 优化方法，可选 'SLSQP', 'L-BFGS-B', 'trust-constr'
            target_roll: 目标roll角度（弧度），如果为None则使用0
        
        Returns:
            dict: 关节角度字典 {'joint_1': angle1, 'joint_2': angle2, ...}
            或 None 如果求解失败
        """
        target = np.array(target_position)
        roll_angle = target_roll if target_roll is not None else 0.0
        
        # 初始猜测
        if initial_guess is None:
            x0 = np.array([0.0, 0.0, 0.0, 0.0])
        else:
            x0 = np.array(initial_guess)
        
        # 目标函数：最小化末端位置与目标位置的距离
        def objective(angles):
            current_pos = self.forward_kinematics(angles)
            error = np.linalg.norm(current_pos - target)
            return error
        
        # 关节限制
        bounds = [
            self.joint_limits['joint_1'],
            self.joint_limits['joint_2'],
            self.joint_limits['joint_3'],
            self.joint_limits['joint_4']
        ]
        
        # 求解
        result = minimize(
            objective,
            x0,
            method=method,
            bounds=bounds,
            options={'maxiter': 1000, 'ftol': 1e-6}
        )
        
        if result.success:
            angles = result.x
            return {
                'joint_1': angles[0],
                'joint_2': angles[1],
                'joint_3': angles[2],
                'joint_4': angles[3],
                'joint_5': roll_angle
            }
        else:
            return None
    
    def solve_ik_multiple_attempts(self, target_position, num_attempts=5, target_roll=None, previous_angles=None):
        """多次尝试IK求解，使用不同的初始猜测
        
        Args:
            target_position: 目标位置 [x, y, z]
            num_attempts: 尝试次数
            target_roll: 目标roll角度（弧度）
            previous_angles: 上一次的关节角度（字典），用于连续性
        
        Returns:
            dict: 最佳的关节角度解
            或 None 如果所有尝试都失败
        """
        best_solution = None
        best_error = float('inf')
        
        target = np.array(target_position)
        
        for i in range(num_attempts):
            # 生成初始猜测
            if i == 0 and previous_angles is not None:
                # 第一次尝试使用上一次的角度（保持连续性）
                initial_guess = [
                    previous_angles.get('joint_1', 0.0),
                    previous_angles.get('joint_2', 0.0),
                    previous_angles.get('joint_3', 0.0),
                    previous_angles.get('joint_4', 0.0)
                ]
            elif i == 1:
                # 第二次尝试使用全0
                initial_guess = [0.0, 0.0, 0.0, 0.0]
            else:
                # 其他尝试使用随机值
                initial_guess = [
                    np.random.uniform(*self.joint_limits['joint_1']),
                    np.random.uniform(*self.joint_limits['joint_2']),
                    np.random.uniform(*self.joint_limits['joint_3']),
                    np.random.uniform(*self.joint_limits['joint_4'])
                ]
            
            solution = self.inverse_kinematics(target_position, initial_guess, target_roll=target_roll)
            
            if solution is not None:
                # 验证解的精度
                achieved_pos = self.forward_kinematics(solution)
                error = np.linalg.norm(achieved_pos - target)
                
                if error < best_error:
                    best_error = error
                    best_solution = solution
                
                # 如果误差足够小，直接返回
                if error < 0.001:  # 1mm误差
                    return best_solution
        
        return best_solution
    
    def get_reachable_workspace(self, num_samples=1000):
        """获取机械臂的可达工作空间
        
        Args:
            num_samples: 采样点数量
        
        Returns:
            numpy array: 可达位置的点云 (N, 3)
        """
        positions = []
        
        for _ in range(num_samples):
            # 随机生成关节角度
            angles = [
                np.random.uniform(*self.joint_limits['joint_1']),
                np.random.uniform(*self.joint_limits['joint_2']),
                np.random.uniform(*self.joint_limits['joint_3']),
                np.random.uniform(*self.joint_limits['joint_4'])
            ]
            
            pos = self.forward_kinematics(angles)
            positions.append(pos)
        
        return np.array(positions)


# 测试和示例
if __name__ == "__main__":
    print("=" * 60)
    print("机械臂逆运动学求解器测试")
    print("=" * 60)
    
    # 创建IK求解器
    ik_solver = RobotIK("robot_template.urdf")
    
    # 测试1: 正运动学
    print("\n测试1: 正运动学")
    print("-" * 60)
    test_angles = [0.0, 0.0, 0.0, 0.0]
    pos = ik_solver.forward_kinematics(test_angles)
    print(f"关节角度: {test_angles}")
    print(f"末端位置: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
    
    # 测试2: 逆运动学
    print("\n测试2: 逆运动学")
    print("-" * 60)
    target = [0.1, 0.1, 0.3]
    print(f"目标位置: {target}")
    
    solution = ik_solver.solve_ik_multiple_attempts(target, num_attempts=10)
    
    if solution:
        print("求解成功!")
        print(f"关节角度:")
        for joint, angle in solution.items():
            if joint != 'joint_5':
                print(f"  {joint}: {angle:.4f} rad ({np.degrees(angle):.2f}°)")
        
        # 验证
        achieved = ik_solver.forward_kinematics(solution)
        error = np.linalg.norm(achieved - np.array(target))
        print(f"\n实际到达位置: [{achieved[0]:.4f}, {achieved[1]:.4f}, {achieved[2]:.4f}]")
        print(f"误差: {error*1000:.2f} mm")
    else:
        print("求解失败，目标位置可能不可达")
    
    # 测试3: 工作空间分析
    print("\n测试3: 工作空间分析")
    print("-" * 60)
    print("采样1000个随机姿态...")
    workspace = ik_solver.get_reachable_workspace(1000)
    
    print(f"X范围: [{workspace[:, 0].min():.3f}, {workspace[:, 0].max():.3f}]")
    print(f"Y范围: [{workspace[:, 1].min():.3f}, {workspace[:, 1].max():.3f}]")
    print(f"Z范围: [{workspace[:, 2].min():.3f}, {workspace[:, 2].max():.3f}]")
    
    max_reach = np.max(np.linalg.norm(workspace, axis=1))
    print(f"最大到达距离: {max_reach:.3f} m")
    
    print("\n" + "=" * 60)
