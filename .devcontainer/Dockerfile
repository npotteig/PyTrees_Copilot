FROM ros:humble-ros-core

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG GHC_VERSION=9.6.3
ARG CABAL_VERSION=3.4

ARG REPO=https://github.com/Copilot-Language/copilot
ARG REPO_NAME=copilot
ARG REPO_COMMIT=835deafa4727c60741337a84cf7b8dce705f6e41

RUN ulimit -n 8192

# Install some initial dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    build-essential \
    software-properties-common \
    python3-pip \
    git \
    sudo \
    python3-colcon-common-extensions \
    xauth \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install PyTrees ROS2 package dependencies
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    ros-humble-py-trees \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-py-trees-ros \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install Haskell and Copilot package depedencies
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    libz-dev \
    pkg-config \
    curl libffi-dev libffi8ubuntu1 libgmp-dev libgmp10 libncurses-dev libncurses5 libtinfo5 \
    gcc g++ make libgmp3-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Add User and update Sudoers List
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd -ms /bin/bash -K MAIL_DIR=/dev/null --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# Set the user
USER ${USER_UID}:${USER_GID}

# Update PATH with cabal and ghcup
ENV PATH="/home/${USERNAME}/.local/bin:/home/${USERNAME}/.cabal/bin:/home/${USERNAME}/.ghcup/bin:$PATH"

RUN echo "export PATH=${PATH}" >> /home/${USERNAME}/.profile

# Download and install ghcup
RUN curl --proto '=https' --tlsv1.2 -sSf https://get-ghcup.haskell.org | sh

# Set the GHC version.
RUN ghcup install ghc ${GHC_VERSION} --set

# Install cabal-install
RUN ghcup install cabal ${CABAL_VERSION} --set

# Update Cabal.
RUN cabal update && cabal new-install cabal-install

# Install Copilot
RUN cd /home/${USERNAME} \
    && git clone $REPO \
    && cd $REPO_NAME \
    && git checkout $REPO_COMMIT \
    && cabal install --lib copilot**/ \
         --constraint="copilot==3.18.1" \
         --constraint="copilot-c99==3.18.1" \
         --constraint="copilot-core==3.18.1" \
         --constraint="copilot-prettyprinter==3.18.1" \
         --constraint="copilot-interpreter==3.18.1" \
         --constraint="copilot-language==3.18.1" \
         --constraint="copilot-libraries==3.18.1" \
         --constraint="copilot-theorem==3.18.1" \
    && echo Success

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws/

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

RUN pip3 install -U pycparser jinja2

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]