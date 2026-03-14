# 机械臂实时渲染器 + 逆运动学

一个基于Pygame/OpenGL的机械臂URDF实时渲染工具，带有逆运动学（IK）求解器，专注于可视化和运动控制。

## 功能特点

- 解析URDF文件并渲染3D模型
- 实时更新关节角度
- 每个连杆使用不同颜色标识
- 支持旋转关节的动态变换
- 交互式视角控制（鼠标旋转、缩放）
- 地面网格和坐标轴参考
- 基于游戏引擎技术，性能稳定流畅
- 60 FPS渲染
- **逆运动学（IK）求解器**
- **目标位置控制**
- **平滑运动插值**

## 安装

```bash
pip install -r requirements.txt
```

## 快速开始

### 1. WebSocket服务器（推荐）

启动WebSocket服务器，通过网络控制机械臂：

**仅虚拟渲染（无真实舵机）：**
```bash
python robot_websocket_server.py
```

**启用真实舵机控制：**
```bash
python robot_websocket_server.py --servo --servo-port COM4
```

参数说明：
- `--host`: 服务器地址（默认: 0.0.0.0）
- `--port`: 服务器端口（默认: 8765）
- `--servo`: 启用舵机控制
- `--servo-port`: 舵机串口（默认: COM4, 115200波特率）

服务器默认监听 `ws://0.0.0.0:8765`

然后可以：
- 使用Web界面：在浏览器打开 `web_client.html`
- 使用Python客户端：运行 `python test_websocket_client.py`
- 使用任何WebSocket客户端发送JSON命令

### 2. 基础渲染演示

```bash
python robot_renderer.py
```

机械臂会自动进行波浪运动演示。

### 3. IK演示

```bash
python robot_ik_demo.py
```

选择演示模式：
- 模式1：自动演示序列 - 机械臂自动移动到多个目标点
- 模式2：网络控制模拟 - 模拟从网络接收目标坐标

### 4. IK求解器测试

```bash
python robot_ik.py
```

测试正运动学、逆运动学和工作空间分析。

## WebSocket API

### 连接

```
ws://localhost:8765
```

### 命令格式

所有命令使用JSON格式。

#### 1. 移动到目标位置

```json
{
  "command": "move",
  "x": 0.1,
  "y": 0.1,
  "z": 0.3,
  "roll": 0.5,
  "duration": 1.5
}
```

参数说明：
- `x`, `y`, `z`: 目标位置（工作空间坐标，米）
- `roll`: 可选，末端执行器roll角度（弧度），控制joint_5旋转
- `duration`: 可选，移动时间（秒）

或使用嵌套格式：

```json
{
  "command": "move",
  "position": {
    "x": 0.1,
    "y": 0.1,
    "z": 0.3
  },
  "roll": 0.5,
  "duration": 2.0
}
```

响应（成功）：
```json
{
  "status": "success",
  "message": "移动完成",
  "target": [0.1, 0.1, 0.3],
  "achieved": [0.1000, 0.1000, 0.3000],
  "error_mm": 0.05,
  "joint_angles": {
    "joint_1": 1.28,
    "joint_2": -0.34,
    "joint_3": 0.99,
    "joint_4": 0.88,
    "joint_5": 0.0
  }
}
```

响应（失败）：
```json
{
  "status": "error",
  "message": "目标位置不可达",
  "target": [1.0, 1.0, 1.0]
}
```

#### 2. 查询状态

```json
{
  "command": "get_status"
}
```

响应：
```json
{
  "status": "ok",
  "is_moving": false,
  "current_position": {
    "x": 0.1,
    "y": 0.1,
    "z": 0.3
  },
  "current_angles": {
    "joint_1": 1.28,
    "joint_2": -0.34,
    "joint_3": 0.99,
    "joint_4": 0.88,
    "joint_5": 0.0
  },
  "statistics": {
    "total_commands": 10,
    "successful_moves": 8,
    "failed_moves": 2
  },
  "connected_clients": 1
}
```

#### 3. 获取当前位置

```json
{
  "command": "get_position"
}
```

响应：
```json
{
  "status": "ok",
  "position": {
    "x": 0.1,
    "y": 0.1,
    "z": 0.3
  }
}
```

#### 4. 设置移动速度

```json
{
  "command": "set_speed",
  "duration": 1.5
}
```

响应：
```json
{
  "status": "ok",
  "message": "移动时间设置为 1.5 秒"
}
```

### 使用示例

#### Python客户端

```python
import asyncio
import websockets
import json

async def control_robot():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        # 移动到目标位置
        command = {
            "command": "move",
            "x": 0.15,
            "y": 0.0,
            "z": 0.25,
            "duration": 2.0
        }
        await websocket.send(json.dumps(command))
        response = await websocket.recv()
        print(json.loads(response))

asyncio.run(control_robot())
```

#### JavaScript客户端

```javascript
const ws = new WebSocket('ws://localhost:8765');

ws.onopen = () => {
    // 移动到目标位置
    const command = {
        command: 'move',
        x: 0.15,
        y: 0.0,
        z: 0.25,
        duration: 2.0
    };
    ws.send(JSON.stringify(command));
};

ws.onmessage = (event) => {
    const response = JSON.parse(event.data);
    console.log(response);
};
```

#### curl测试（使用websocat）

```bash
# 安装 websocat: https://github.com/vi/websocat
echo '{"command":"move","x":0.15,"y":0.0,"z":0.25}' | websocat ws://localhost:8765
```

## 舵机控制

### 硬件要求

- LewanSoul/Hiwonder LX-16A舵机
- USB转TTL串口模块
- 串口协议: `#XXXPYYYYTZZZ!`
  - XXX: 舵机ID (000-255)
  - YYYY: 位置 (0500-2500)
  - ZZZ: 时间 (毫秒)

### 舵机ID映射

默认配置（可在`servo_controller.py`中修改）：
- joint_1 (底座旋转): ID 0
- joint_2 (肩部): ID 1
- joint_3 (肘部): ID 2
- joint_4 (腕部): ID 3
- joint_5 (末端旋转): ID 4

### 测试舵机

```bash
python servo_controller.py
```

这会测试所有舵机的连接和运动。

## 使用示例

### 基础渲染

```python
from robot_renderer import RobotRenderer

# 创建渲染器
renderer = RobotRenderer("robot_template.urdf")

# 更新关节角度（弧度）
angles = {
    'joint_1': 0.5,   # 底座旋转
    'joint_2': 0.3,   # 肩部
    'joint_3': -0.5,  # 肘部
    'joint_4': 0.2,   # 腕部
    'joint_5': 0.0    # 末端旋转
}
renderer.update_joint_angles(angles)

# 运行渲染循环
renderer.run()
```

### 使用IK控制

```python
from robot_ik import RobotIK

# 创建IK求解器
ik_solver = RobotIK("robot_template.urdf")

# 目标位置（米）
target_position = [0.1, 0.1, 0.3]

# 求解IK
solution = ik_solver.solve_ik_multiple_attempts(target_position, num_attempts=10)

if solution:
    print("关节角度:", solution)
    # solution = {'joint_1': 1.28, 'joint_2': -0.34, ...}
```

### 结合IK和渲染

```python
import time
import threading
from robot_renderer import RobotRenderer
from robot_ik import RobotIK

renderer = RobotRenderer("robot_template.urdf")
ik_solver = RobotIK("robot_template.urdf")

def move_to_target(target_position):
    # 求解IK
    solution = ik_solver.solve_ik_multiple_attempts(target_position)
    if solution:
        # 更新渲染器
        renderer.update_joint_angles(solution)
        return True
    return False

# 在后台线程控制运动
def control_loop():
    time.sleep(2)
    targets = [[0.15, 0.0, 0.25], [0.0, 0.15, 0.25], [0.1, 0.1, 0.3]]
    for target in targets:
        move_to_target(target)
        time.sleep(2)

threading.Thread(target=control_loop, daemon=True).start()
renderer.run()
```

### 从网络接收目标位置

```python
import socket
import json
import threading
from robot_renderer import RobotRenderer
from robot_ik import RobotIK

renderer = RobotRenderer("robot_template.urdf")
ik_solver = RobotIK("robot_template.urdf")

def network_receiver():
    # 创建socket服务器
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 5000))
    server.listen(1)
    
    while True:
        conn, addr = server.accept()
        data = conn.recv(1024).decode()
        
        # 解析JSON: {"x": 0.1, "y": 0.1, "z": 0.3}
        target = json.loads(data)
        position = [target['x'], target['y'], target['z']]
        
        # 求解并移动
        solution = ik_solver.solve_ik_multiple_attempts(position)
        if solution:
            renderer.update_joint_angles(solution)
            conn.send(b"OK")
        else:
            conn.send(b"UNREACHABLE")
        
        conn.close()

threading.Thread(target=network_receiver, daemon=True).start()
renderer.run()
```

## API说明

### RobotRenderer类

#### `__init__(urdf_file)`
创建渲染器实例

#### `update_joint_angles(angles_dict)`
更新关节角度
- `angles_dict`: 字典，键为关节名称，值为角度（弧度）

#### `run()`
运行主渲染循环（阻塞调用）

### RobotIK类

#### `__init__(urdf_file)`
创建IK求解器实例

#### `forward_kinematics(joint_angles)`
正运动学：计算末端执行器位置
- `joint_angles`: 关节角度（字典或列表）
- 返回: [x, y, z] 位置（米）

#### `inverse_kinematics(target_position, initial_guess=None)`
逆运动学：计算到达目标位置的关节角度
- `target_position`: 目标位置 [x, y, z]（米）
- `initial_guess`: 初始猜测（可选）
- 返回: 关节角度字典，或None（失败）

#### `solve_ik_multiple_attempts(target_position, num_attempts=5)`
多次尝试IK求解（推荐使用）
- `target_position`: 目标位置 [x, y, z]（米）
- `num_attempts`: 尝试次数
- 返回: 最佳关节角度解，或None（失败）

#### `get_reachable_workspace(num_samples=1000)`
获取机械臂可达工作空间
- 返回: numpy数组 (N, 3) 可达位置点云

## 机械臂配置

当前URDF配置：
- 5个旋转关节
- IK控制关节0-3（joint_1到joint_4）
- 关节4（joint_5）固定为0
- 关节角度范围：±135度（±2.356弧度）
- 关节1: 底座水平旋转（Z轴）
- 关节2-4: 俯仰运动（X轴）
- 关节5: 末端旋转（Z轴）

工作空间（近似）：
- X范围: [-0.28, 0.27] 米
- Y范围: [-0.21, 0.35] 米
- Z范围: [-0.13, 0.41] 米
- 最大到达距离: ~0.42 米

## 文件说明

- `robot_template.urdf` - 机械臂URDF模型文件
- `robot_renderer.py` - 核心渲染器类（Pygame/OpenGL）
- `robot_ik.py` - 逆运动学求解器
- `robot_ik_demo.py` - IK演示程序
- `robot_websocket_server.py` - WebSocket服务器（主程序）
- `test_websocket_client.py` - Python测试客户端
- `web_client.html` - Web控制面板
- `example_usage.py` - 基础使用示例

## 交互控制

- **左键拖动**: 旋转视角
- **滚轮**: 缩放
- **ESC/Q**: 退出

## 技术特点

- 使用Pygame + PyOpenGL实现
- 硬件加速的3D渲染
- 正确的深度缓冲和光照
- 60 FPS流畅渲染
- 低CPU占用
- 基于scipy的数值优化IK求解
- 多次随机初始化提高求解成功率
- 平滑运动插值

## 注意事项

- 角度单位为弧度
- 位置单位为米
- 坐标系：X前，Y左，Z上
- IK求解可能需要0.1-1秒
- 建议使用 `solve_ik_multiple_attempts` 提高成功率
- 目标位置超出工作空间会返回None

## 网络协议示例

发送目标位置到机械臂（JSON格式）：

```json
{
  "x": 0.1,
  "y": 0.1,
  "z": 0.3
}
```

响应：
- `"OK"` - 成功到达
- `"UNREACHABLE"` - 目标不可达
