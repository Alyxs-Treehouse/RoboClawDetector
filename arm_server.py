"""
机械臂 HTTP API 服务器

包装 ArmActions，通过 HTTP 接口供 OpenClaw 调用。
启动时打开串口，全程保持连接。

启动: python arm_server.py
API:  POST /action/nod, GET /actions, GET /health
"""

import threading
from flask import Flask, jsonify
from arm_actions import ArmActions

app = Flask(__name__)

# ─── 全局状态 ──────────────────────────────────────

arm = None          # ArmActions 实例，在 main 中初始化
busy_lock = threading.Lock()


# ─── API 路由 ──────────────────────────────────────

@app.route('/health', methods=['GET'])
def health():
    """健康检查"""
    return jsonify({"status": "ok"})


@app.route('/actions', methods=['GET'])
def list_actions():
    """列出所有可用动作"""
    return jsonify({
        "actions": list(ACTION_MAP.keys())
    })


@app.route('/action/<action_name>', methods=['POST'])
def execute_action(action_name):
    """执行指定动作"""
    if action_name not in ACTION_MAP:
        return jsonify({
            "status": "error",
            "message": f"未知动作: {action_name}",
            "available": list(ACTION_MAP.keys())
        }), 404

    # 尝试获取锁（非阻塞）
    if not busy_lock.acquire(blocking=False):
        return jsonify({
            "status": "busy",
            "message": "正在执行其他动作，请稍后重试"
        }), 429

    try:
        ACTION_MAP[action_name]()
        return jsonify({
            "status": "ok",
            "action": action_name
        })
    except Exception as e:
        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500
    finally:
        busy_lock.release()


# ─── 启动 ──────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='机械臂 HTTP API 服务器')
    parser.add_argument('--port', type=int, default=5050, help='HTTP 端口')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--serial', default='/dev/tty.usbserial-210', help='串口设备')
    args = parser.parse_args()

    # 初始化机械臂（打开串口）
    print("=" * 50)
    print("机械臂 HTTP API 服务器")
    print("=" * 50)
    arm = ArmActions(port=args.serial)

    # 动作映射（必须在 arm 初始化后创建）
    ACTION_MAP = {
        "center": arm.center,
        "nod": arm.nod,
        "shake_head": arm.shake_head,
        "shrug": arm.shrug,
        "grab_bag": arm.grab_bag,
        "point_forward": arm.point_forward,
        "point_down": arm.point_down,
        "shoot": arm.shoot,
        "kick": arm.kick,
        "dance": arm.dance,
    }

    print(f"\n可用动作: {', '.join(ACTION_MAP.keys())}")
    print(f"API 地址: http://{args.host}:{args.port}")
    print(f"示例: curl -X POST http://localhost:{args.port}/action/nod")
    print("=" * 50)

    app.run(host=args.host, port=args.port, debug=False)
