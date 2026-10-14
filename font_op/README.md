# 麒麟端前端部署

访问：http://10.186.109.146:8888/

本目录目前只有 `dist` 打包产物，没有前端源码和 package.json。这些浏览器资源不依赖 Windows，不需要在麒麟安装 Node.js。

在项目根目录执行 `GUI=0 BUILD_JOBS=8 ROS_IMAGE=xczs-ros-base:arm64 bash ./start`，会将本目录打包进仿真镜像，加载 nginx.conf，并映射宿主机 8888 端口。后续随 xczs-simulation 容器一起启停。

浏览器通过同源 `/api/` 请求接口及 WebSocket，Nginx 去掉 `/api/` 前缀并转发到容器内 `127.0.0.1:8090`。这里的回环地址是后端服务地址，不是浏览器访问地址；服务器 IP 改变时使用新的页面网址即可。

更新前端时替换 dist，保留此 Linux Nginx 配置，再运行根目录 start 重建部署。当前打包产物中的旧 Windows IP 错误提示已改为通用服务器提示，后续从源码重新打包时也应同步修改。
