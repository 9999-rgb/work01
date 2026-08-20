"""资产目录与选择持久化（SQLAlchemy）。

模型与 store 复用应用层 SQLAlchemy 栈；同步 session 供 Web（线程池）、CLI 与
启动脚本共用，目录与选择与 auth 用户同库（``XCZS_DATABASE_URL``）。
"""
