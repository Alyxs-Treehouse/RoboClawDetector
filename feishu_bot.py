"""
飞书聊天机器人（长连接模式 + Claude AI）

收到用户消息时：
1. 调用 Claude API 生成智能回答
2. 如果回答含肯定语气，同时让机械臂点头

无需公网地址，通过飞书官方 SDK 长连接接收事件。

启动：
  python feishu_bot.py
"""

import json
import os
import threading
import requests
from openai import OpenAI
import lark_oapi as lark

# 屏蔽 socks 代理（httpx 不支持），保留 http 代理
os.environ.pop("ALL_PROXY", None)
os.environ.pop("all_proxy", None)
from lark_oapi.api.im.v1 import *

# ─── 飞书应用凭证 ─────────────────────────────────────────────
APP_ID = "cli_a9343243abf8dcef"
APP_SECRET = "MHR81CP1xKHi87ShhkPL4elpCJBaFoJ3"

# ─── OpenAI API ──────────────────────────────────────────────
# 从环境变量读取，启动前执行: export OPENAI_API_KEY=sk-...
openai_client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])

# ─── arm_server 地址 ─────────────────────────────────────────
ARM_SERVER_URL = "http://localhost:5050"

# ─── 飞书客户端 ───────────────────────────────────────────────
feishu = lark.Client.builder() \
    .app_id(APP_ID) \
    .app_secret(APP_SECRET) \
    .build()

# 每个用户的对话历史（多轮对话）
user_histories: dict[str, list] = {}
history_lock = threading.Lock()

# 肯定词列表，触发点头
NOD_KEYWORDS = ["是", "对", "好", "可以", "没问题", "当然", "确实", "正确", "同意", "赞同", "没错"]


# ─── Claude 对话 ──────────────────────────────────────────────

def ask_gemini(user_id: str, user_text: str) -> str:
    """调用 OpenAI API，保持多轮对话历史"""
    with history_lock:
        if user_id not in user_histories:
            user_histories[user_id] = []
        user_histories[user_id].append({"role": "user", "content": user_text})
        history = user_histories[user_id].copy()

    messages = [{"role": "system", "content": "你是一个友好的助手，回答简洁清晰。"}] + history

    response = openai_client.chat.completions.create(
        model="gpt-4o",
        messages=messages,
    )
    reply = response.choices[0].message.content

    with history_lock:
        user_histories[user_id].append({"role": "assistant", "content": reply})
        if len(user_histories[user_id]) > 20:
            user_histories[user_id] = user_histories[user_id][-20:]

    return reply


def should_nod(text: str) -> bool:
    """判断回答是否含肯定语气，决定是否点头"""
    return any(kw in text for kw in NOD_KEYWORDS)


# ─── 工具函数 ─────────────────────────────────────────────────

def reply_message(message_id: str, text: str):
    """回复飞书消息"""
    request = ReplyMessageRequest.builder() \
        .message_id(message_id) \
        .request_body(ReplyMessageRequestBody.builder()
                      .content(json.dumps({"text": text}))
                      .msg_type("text")
                      .build()) \
        .build()
    resp = feishu.im.v1.message.reply(request)
    if not resp.success():
        print(f"[bot] 回复失败: {resp.code} {resp.msg}")


def trigger_nod():
    """调用 arm_server 执行点头动作"""
    try:
        resp = requests.post(f"{ARM_SERVER_URL}/action/nod", timeout=10)
        data = resp.json()
        if data.get("status") == "ok":
            print("[arm] 点头成功")
        else:
            print(f"[arm] 点头失败: {data}")
    except requests.exceptions.ConnectionError:
        print(f"[arm] 无法连接 arm_server（{ARM_SERVER_URL}），机械臂未启动")
    except Exception as e:
        print(f"[arm] 异常: {e}")


def handle(message_id: str, user_id: str, user_text: str):
    """处理一条消息：问 Claude → 回复 → 判断是否点头"""
    print(f"[bot] 用户({user_id}): {user_text}")
    try:
        answer = ask_gemini(user_id, user_text)
    except Exception as e:
        answer = f"抱歉，出错了：{e}"

    print(f"[bot] Claude 回答: {answer}")
    reply_message(message_id, answer)

    if should_nod(answer):
        threading.Thread(target=trigger_nod, daemon=True).start()


# ─── 消息事件处理 ─────────────────────────────────────────────

def on_message_receive(data: P2ImMessageReceiveV1):
    sender = data.event.sender
    if sender.sender_type != "user":
        return

    msg = data.event.message
    message_id = msg.message_id
    user_id = sender.sender_id.open_id

    # 解析文本内容
    try:
        content = json.loads(msg.content)
        user_text = content.get("text", "").strip()
    except Exception:
        user_text = ""

    if not user_text:
        return

    # 在新线程里处理，避免阻塞事件循环
    threading.Thread(target=handle, args=(message_id, user_id, user_text), daemon=True).start()


# ─── 启动长连接 ───────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 50)
    print("飞书机器人服务（长连接 + OpenAI）")
    print(f"App ID:     {APP_ID}")
    print(f"arm_server: {ARM_SERVER_URL}")
    print("=" * 50)

    event_handler = lark.EventDispatcherHandler.builder("", "") \
        .register_p2_im_message_receive_v1(on_message_receive) \
        .build()

    ws_client = lark.ws.Client(
        APP_ID,
        APP_SECRET,
        event_handler=event_handler,
        log_level=lark.LogLevel.INFO,
    )

    ws_client.start()
