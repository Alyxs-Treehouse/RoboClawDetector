import xml.etree.ElementTree as ET
import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

class RobotRenderer:
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.links, self.joints = self.parse_urdf(urdf_file)
        self.joint_angles = {name: 0.0 for name in self.joints.keys()}
        
        # 初始化Pygame和OpenGL
        pygame.init()
        self.display = (1200, 900)
        pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption('Robot Arm Renderer - Pygame/OpenGL')
        
        # 设置OpenGL
        self.setup_opengl()
        
        # 相机控制
        self.camera_distance = 1.0
        self.camera_rotation_x = 30
        self.camera_rotation_y = 45
        self.mouse_down = False
        self.last_mouse_pos = (0, 0)
        
    def setup_opengl(self):
        """设置OpenGL参数"""
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # 设置光源
        glLight(GL_LIGHT0, GL_POSITION, (1, 1, 1, 0))
        glLight(GL_LIGHT0, GL_AMBIENT, (0.3, 0.3, 0.3, 1))
        glLight(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1))
        
        # 设置投影
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (self.display[0] / self.display[1]), 0.01, 50.0)
        glMatrixMode(GL_MODELVIEW)
        
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
                
                # 获取几何信息
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
                
                # 获取origin信息
                xyz = [0, 0, 0]
                rpy = [0, 0, 0]
                if origin is not None:
                    if origin.get('xyz'):
                        xyz = [float(x) for x in origin.get('xyz').split()]
                    if origin.get('rpy'):
                        rpy = [float(x) for x in origin.get('rpy').split()]
                
                # 获取颜色
                material = visual.find('material')
                color = [0.5, 0.5, 0.5]
                if material is not None:
                    color_elem = material.find('color')
                    if color_elem is not None:
                        rgba = [float(x) for x in color_elem.get('rgba').split()]
                        color = rgba[:3]
                
                links[name] = {
                    'geometry': geom_info,
                    'origin': xyz,
                    'rpy': rpy,
                    'color': color
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
    
    def calculate_transforms(self):
        """计算每个link的变换矩阵"""
        transforms = {}
        
        # 基座变换
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
                
                # 关节的局部变换
                joint_offset = np.array(joint_info['origin'])
                joint_rpy = joint_info['rpy']
                joint_rot = self.rpy_to_rotation_matrix(joint_rpy)
                
                # 关节角度旋转
                if joint_info['type'] == 'revolute':
                    angle = self.joint_angles.get(joint_name, 0.0)
                    axis = joint_info['axis']
                    angle_rot = self.rotation_matrix(axis, angle)
                    joint_rot = joint_rot @ angle_rot
                
                # 计算子link的全局变换
                child_pos = parent_pos + parent_rot @ joint_offset
                child_rot = parent_rot @ joint_rot
                
                transforms[child] = {
                    'position': child_pos,
                    'rotation': child_rot
                }
        
        return transforms
    
    def draw_box(self, size):
        """绘制长方体"""
        x, y, z = size[0]/2, size[1]/2, size[2]/2
        
        vertices = [
            [-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],  # 底面
            [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]       # 顶面
        ]
        
        faces = [
            [0, 1, 2, 3],  # 底面
            [4, 5, 6, 7],  # 顶面
            [0, 1, 5, 4],  # 前面
            [2, 3, 7, 6],  # 后面
            [0, 3, 7, 4],  # 左面
            [1, 2, 6, 5]   # 右面
        ]
        
        normals = [
            [0, 0, -1], [0, 0, 1],
            [0, -1, 0], [0, 1, 0],
            [-1, 0, 0], [1, 0, 0]
        ]
        
        glBegin(GL_QUADS)
        for i, face in enumerate(faces):
            glNormal3fv(normals[i])
            for vertex_idx in face:
                glVertex3fv(vertices[vertex_idx])
        glEnd()
    
    def draw_cylinder(self, radius, length, slices=20):
        """绘制圆柱体"""
        # 绘制侧面
        glBegin(GL_QUAD_STRIP)
        for i in range(slices + 1):
            angle = 2 * np.pi * i / slices
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            glNormal3f(np.cos(angle), np.sin(angle), 0)
            glVertex3f(x, y, 0)
            glVertex3f(x, y, length)
        glEnd()
        
        # 绘制底面
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, -1)
        glVertex3f(0, 0, 0)
        for i in range(slices + 1):
            angle = 2 * np.pi * i / slices
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            glVertex3f(x, y, 0)
        glEnd()
        
        # 绘制顶面
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0, 0, 1)
        glVertex3f(0, 0, length)
        for i in range(slices, -1, -1):
            angle = 2 * np.pi * i / slices
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            glVertex3f(x, y, length)
        glEnd()
    
    def draw_grid(self):
        """绘制地面网格"""
        glDisable(GL_LIGHTING)
        glColor3f(0.7, 0.7, 0.7)
        glBegin(GL_LINES)
        
        grid_size = 0.8
        grid_step = 0.1
        
        for i in np.arange(-grid_size, grid_size + grid_step, grid_step):
            glVertex3f(i, -grid_size, 0)
            glVertex3f(i, grid_size, 0)
            glVertex3f(-grid_size, i, 0)
            glVertex3f(grid_size, i, 0)
        
        glEnd()
        glEnable(GL_LIGHTING)
    
    def draw_axes(self):
        """绘制坐标轴"""
        glDisable(GL_LIGHTING)
        glLineWidth(3)
        glBegin(GL_LINES)
        
        # X轴 - 红色
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0.2, 0, 0)
        
        # Y轴 - 绿色
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0.2, 0)
        
        # Z轴 - 蓝色
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 0.2)
        
        glEnd()
        glLineWidth(1)
        glEnable(GL_LIGHTING)
    
    def render(self):
        """渲染场景"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # 设置相机
        gluLookAt(
            self.camera_distance * np.cos(np.radians(self.camera_rotation_y)) * np.cos(np.radians(self.camera_rotation_x)),
            self.camera_distance * np.sin(np.radians(self.camera_rotation_y)) * np.cos(np.radians(self.camera_rotation_x)),
            self.camera_distance * np.sin(np.radians(self.camera_rotation_x)),
            0, 0, 0.2,
            0, 0, 1
        )
        
        # 绘制网格和坐标轴
        self.draw_grid()
        self.draw_axes()
        
        # 计算所有变换
        transforms = self.calculate_transforms()
        
        # 绘制每个link
        link_order = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5']
        
        for link_name in link_order:
            if link_name in self.links and link_name in transforms:
                link_info = self.links[link_name]
                transform = transforms[link_name]
                pos = transform['position']
                rot = transform['rotation']
                
                origin_offset = np.array(link_info['origin'])
                origin_rpy = link_info['rpy']
                origin_rot = self.rpy_to_rotation_matrix(origin_rpy)
                
                # 计算最终变换
                center = pos + rot @ origin_offset
                final_rot = rot @ origin_rot
                
                # 应用变换
                glPushMatrix()
                
                # 平移
                glTranslatef(center[0], center[1], center[2])
                
                # 旋转（转换为OpenGL格式）
                rot_matrix = np.eye(4)
                rot_matrix[:3, :3] = final_rot
                glMultMatrixf(rot_matrix.T.flatten())
                
                # 设置颜色
                color = link_info['color']
                glColor3f(color[0], color[1], color[2])
                
                # 绘制几何体
                geom = link_info['geometry']
                if geom['type'] == 'box':
                    self.draw_box(geom['size'])
                elif geom['type'] == 'cylinder':
                    glTranslatef(0, 0, -geom['length']/2)
                    self.draw_cylinder(geom['radius'], geom['length'])
                
                glPopMatrix()
        
        pygame.display.flip()
    
    def handle_events(self):
        """处理事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 左键
                    self.mouse_down = True
                    self.last_mouse_pos = pygame.mouse.get_pos()
                elif event.button == 4:  # 滚轮向上
                    self.camera_distance *= 0.9
                elif event.button == 5:  # 滚轮向下
                    self.camera_distance *= 1.1
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.mouse_down = False
            elif event.type == pygame.MOUSEMOTION:
                if self.mouse_down:
                    current_pos = pygame.mouse.get_pos()
                    dx = current_pos[0] - self.last_mouse_pos[0]
                    dy = current_pos[1] - self.last_mouse_pos[1]
                    self.camera_rotation_y += dx * 0.5
                    self.camera_rotation_x += dy * 0.5
                    self.camera_rotation_x = max(-89, min(89, self.camera_rotation_x))
                    self.last_mouse_pos = current_pos
        
        return True
    
    def update_joint_angles(self, angles_dict):
        """更新关节角度
        
        Args:
            angles_dict: 字典，键为关节名称，值为角度（弧度）
        """
        self.joint_angles.update(angles_dict)
    
    def run(self):
        """运行主循环"""
        clock = pygame.time.Clock()
        running = True
        
        while running:
            running = self.handle_events()
            self.render()
            clock.tick(60)  # 60 FPS
        
        pygame.quit()


# 示例使用
if __name__ == "__main__":
    import time
    
    print("=" * 60)
    print("机械臂实时渲染器 (Pygame/OpenGL)")
    print("=" * 60)
    print("鼠标操作:")
    print("  - 左键拖动: 旋转视角")
    print("  - 滚轮: 缩放")
    print("  - ESC/Q: 退出")
    print("=" * 60)
    print("开始模拟关节运动...")
    print("=" * 60)
    
    renderer = RobotRenderer("robot_template.urdf")
    
    # 在单独的线程中更新角度
    import threading
    
    def update_angles():
        t = 0
        while True:
            angles = {
                'joint_1': 0.5 * np.sin(t),
                'joint_2': 0.3 * np.sin(t * 1.5),
                'joint_3': 0.4 * np.sin(t * 2),
                'joint_4': 0.2 * np.sin(t * 2.5),
                'joint_5': 0.6 * np.sin(t * 0.8)
            }
            renderer.update_joint_angles(angles)
            t += 0.05
            time.sleep(0.05)
    
    angle_thread = threading.Thread(target=update_angles, daemon=True)
    angle_thread.start()
    
    renderer.run()
