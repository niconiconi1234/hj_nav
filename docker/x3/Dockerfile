FROM yahboomtechnology/ros-foxy:4.0.5
WORKDIR /root/yahboomcar_ros2_ws/yahboomcar_ws
# ROS_DOMAIN_ID，这里先写死，后续可以在docker run时指定
ENV ROS_DOMAIN_ID=32
RUN mkdir -p src/hj_nav
# 导航参数
COPY docker/x3/dwa_nav_params.yaml src/yahboomcar_nav/params
# 湾谷地图
COPY docker/map/wangu/* src/yahboomcar_nav/maps
# .bashrc，清除了厂商提供的.bashrc内写死的ROS_DOMAIN_ID，防止多台机器在同一局域网下运行时冲突
COPY docker/x3/.bashrc /root/.bashrc
# 代码
COPY . src/hj_nav
# 安装依赖
RUN bash -c "pip3 install -r src/hj_nav/nav_http_server/requirements.txt"
RUN bash -c "pip3 install -r src/hj_nav/red_detector/requirements.txt"
# 编译
RUN bash -c "colcon build"