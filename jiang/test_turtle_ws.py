#!/usr/bin/env python3
"""Zenoh WS 连接验证 — 订阅 turtle1/cmd_vel/json"""
import time
import json

def now():
    return time.strftime("%H:%M:%S")

def main():
    from zenoh import Config, open as zopen

    config = Config.from_json5(
        '{"mode":"client","connect":{"endpoints":["ws/localhost:10000"]}}'
    )
    print(f"[{now()}] WS 连接 ws/localhost:10000 ...")

    try:
        session = zopen(config)
    except Exception as e:
        print(f"[{now()}] ✗ 连接失败: {e}")
        return
    print(f"[{now()}] ✓ 连接成功")

    # 订阅 turtle1/cmd_vel/json
    topic = "turtle1/cmd_vel/json"
    print(f"[{now()}] 订阅 '{topic}' ...")

    count = [0]

    def on_sample(sample):
        count[0] += 1
        ts = now()
        ke = str(sample.key_expr)
        payload = sample.payload.to_bytes()
        try:
            text = sample.payload.to_string()
            obj = json.loads(text)
            pretty = json.dumps(obj, ensure_ascii=False, indent=2)
            print(f"[{ts}] #{count[0]} {ke} (JSON)\n{pretty}\n", flush=True)
        except Exception:
            print(f"[{ts}] #{count[0]} {ke} ({len(payload)}B CDR) "
                  f"hex={payload[:32].hex()}", flush=True)

    sub = session.declare_subscriber(topic, on_sample)
    print(f"[{now()}] 等待数据 (Ctrl+C 退出)...\n")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n[{now()}] 断开")
    finally:
        sub.undeclare()
        session.close()

if __name__ == "__main__":
    main()
