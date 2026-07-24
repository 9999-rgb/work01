#!/usr/bin/env python3
"""快速验证 Zenoh — 用 eclipse-zenoh 客户端连接并订阅数据"""
import time
import json

def now():
    return time.strftime("%H:%M:%S")

def main():
    from zenoh import Config, open as zopen

    ws_url = "ws://localhost:10000"
    print(f"[{now()}] 连接 {ws_url} ...")
    # 先试 TCP (默认支持)，再试 WS client 模式
    endpoints = [
        "tcp/localhost:7447",
        "ws://localhost:10000",
    ]
    session = None
    for ep in endpoints:
        try:
            mode = "client" if ep.startswith("ws") else "peer"
            config = Config.from_json5(
                f'{{"mode":"{mode}","connect":{{"endpoints":["{ep}"]}}}}'
            )
            session = zopen(config)
            print(f"[{now()}] ✓ 连接成功 ({ep}, mode={mode})")
            break
        except Exception as e:
            print(f"[{now()}] {ep} → {e}")
    if session is None:
        print(f"[{now()}] ✗ 所有端点连接失败")
        return

    # 列出所有 ROS2 转换话题 (通配符订阅)
    wildcard = "**/json"
    print(f"[{now()}] 订阅通配符 '{wildcard}' ...\n")

    def on_sample(sample):
        ts = now()
        ke = sample.key_expr
        try:
            payload = json.loads(sample.payload.to_string())
            pretty = json.dumps(payload, ensure_ascii=False, indent=2)
            if len(pretty) > 400:
                pretty = pretty[:400] + "\n... (truncated)"
        except Exception:
            pretty = sample.payload.to_string()[:300]
        print(f"[{ts}] {ke}\n{pretty}\n")

    sub = session.declare_subscriber(wildcard, on_sample)

    try:
        print(f"[{now()}] 等待数据 (Ctrl+C 退出)...\n")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n[{now()}] 断开")
    finally:
        sub.undeclare()
        session.close()

if __name__ == "__main__":
    main()
