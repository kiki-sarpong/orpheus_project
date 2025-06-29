
ARG BASE_IMAGE=ros:jazzy
FROM ${BASE_IMAGE}

ARG NEW_USER=kiki_dev
ENV WS_DIR=/home/${NEW_USER}

USER root
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && apt-get install -y \
    tree \
    ros-${ROS_DISTRO}-imu-tools \
    # This remembers to clean the apt cache after a run for size reduction
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add the user with sudo privileges
RUN adduser --disabled-password --gecos '' ${NEW_USER} \
&& echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
&& adduser ${NEW_USER} sudo


USER ${NEW_USER}
WORKDIR $WS_DIR
ENV ORPHEUS_DIR=${WS_DIR}/orpheus_ocean

# Matches build context | Cpy the contents
COPY orpheus_ocean ${ORPHEUS_DIR}
# Handle permissions here.
RUN sudo chown -R ${NEW_USER}:${NEW_USER} ${ORPHEUS_DIR}

# Build the package
RUN /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash \
                && cd ${ORPHEUS_DIR} \
                && colcon build"

# Setup ROS2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${NEW_USER}/.bashrc
RUN echo "source ${ORPHEUS_DIR}/install/local_setup.bash" >> /home/${NEW_USER}/.bashrc

CMD ["/bin/bash"]