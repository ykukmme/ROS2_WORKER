ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN for source_file in /etc/apt/sources.list /etc/apt/sources.list.d/*.sources /etc/apt/sources.list.d/*.list; do \
      if [[ -f "$source_file" ]]; then \
        sed -i -E 's@http://(archive|security)\.ubuntu\.com/ubuntu@https://\1.ubuntu.com/ubuntu@g' "$source_file"; \
      fi; \
    done && apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates build-essential python3-colcon-common-extensions python3-venv \
    python3-numpy python3-opencv python3-yaml libgl1 libglib2.0-0 \
    ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-moveit-msgs ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-shape-msgs ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-rosidl-default-generators ros-${ROS_DISTRO}-rosidl-runtime-py \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-urdf ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Official interfaces and mesh assets, fixed to reviewed revisions.
RUN mkdir -p /opt/vendor_ws/src && \
    git init /opt/vendor_ws/src/robotis_interfaces && \
    git -C /opt/vendor_ws/src/robotis_interfaces remote add origin https://github.com/ROBOTIS-GIT/robotis_interfaces.git && \
    git -C /opt/vendor_ws/src/robotis_interfaces fetch --depth 1 origin c79afb6aa8d0900401ec38ecdb60b1a104929997 && \
    git -C /opt/vendor_ws/src/robotis_interfaces checkout --detach FETCH_HEAD && \
    git init /tmp/ai_worker && \
    git -C /tmp/ai_worker remote add origin https://github.com/ROBOTIS-GIT/ai_worker.git && \
    git -C /tmp/ai_worker fetch --depth 1 origin be8ed62384076bd3979ce55b6d5313f435f81475 && \
    git -C /tmp/ai_worker checkout --detach FETCH_HEAD && \
    cp -a /tmp/ai_worker/ffw_description /opt/vendor_ws/src/ && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /opt/vendor_ws && colcon build --packages-select robotis_interfaces ffw_description && \
    rm -rf /tmp/ai_worker

ARG TORCH_INDEX_URL=https://download.pytorch.org/whl/cpu
RUN python3 -m venv --system-site-packages /opt/worker_venv && \
    /opt/worker_venv/bin/python -m pip install --no-cache-dir torch==2.14.1 torchvision==0.29.2 --index-url "${TORCH_INDEX_URL}" && \
    /opt/worker_venv/bin/python -m pip install --no-cache-dir 'numpy==1.26.4' 'opencv-python==4.11.0.86' 'ultralytics==8.4.199'

ENV WORKER_DEFAULT_WEIGHTS=/opt/worker/models/yolo26n.pt \
    WORKER_PYTHON=/opt/worker_venv/bin/python \
    WORKER_CONFIG=/data/config/live.local.yaml \
    WORKER_DEVICE=auto WORKER_IN_CONTAINER=1 HOME=/data PYTHONUNBUFFERED=1
RUN mkdir -p /opt/worker/models && cd /opt/worker/models && \
    /opt/worker_venv/bin/python -c "from ultralytics import YOLO; model=YOLO('yolo26n.pt'); assert model.task=='detect'"

WORKDIR /opt/worker
COPY ai_worker_interfaces ./ai_worker_interfaces
COPY ai_worker_sorting ./ai_worker_sorting
COPY scripts/build_ros2.sh scripts/worker.sh ./scripts/
COPY docker/entrypoint.sh /opt/worker/entrypoint.sh
# Do not seed new runtime volumes with settings generated during the build.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/vendor_ws/install/setup.bash && \
    bash scripts/build_ros2.sh /opt/worker_ws && \
    source /opt/worker_ws/install/ros2_worker/setup.bash && \
    /opt/worker_venv/bin/python -c 'import rclpy, cv_bridge, vision_msgs.msg, robotis_interfaces.msg, ultralytics; from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory("ffw_description"))' && \
    chmod +x /opt/worker/entrypoint.sh && \
    rm -rf /data && mkdir -m 1777 /data
USER 1000:1000
ENTRYPOINT ["/opt/worker/entrypoint.sh"]
CMD ["menu"]
