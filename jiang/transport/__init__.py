"""外部系统传输桥接层：Zenoh CDR→JSON 代理、Zenoh→SSE、传感器流（MJPEG/WebSocket）。

该层只做「外部消息 ↔ 任务层」的桥接与格式转换，不含业务逻辑：
- ``sse_bridge.py``/``zenoh_session.py``/``zenoh_key.py``：Zenoh 客户端与 SSE 发布。
- ``zenoh_proxy/`` + ``run_xczs_proxy.py``：CDR→JSON 代理（浏览器监控面板数据源）。
- ``sensor_bridge/``：相机/LiDAR 传感器流的 ROS 订阅与浏览器格式转换。

运行入口（在 ``jiang/`` 目录下）：``python3 -m transport.run_xczs_proxy``。
"""
