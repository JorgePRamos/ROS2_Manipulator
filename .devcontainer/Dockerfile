FROM osrf/ros:jazzy-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Required repos
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe -y

# ROS 2 GPG key
RUN sudo apt update && sudo apt install curl -y && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install DEV tools
RUN sudo apt update && sudo apt install ros-dev-tools -y

# Upgrade and update
RUN sudo apt update -y && \
    sudo apt upgrade -y

# ROS2 packages
RUN sudo apt-get install ros-jazzy-joint-state-publisher-gui -y && \
    sudo apt-get install ros-jazzy-xacro -y && \
    sudo apt-get install ros-jazzy-ros-gz* -y && \
    sudo apt-get install ros-jazzy-ros2-controllers -y && \
    sudo apt-get install ros-jazzy-ros2-controllers -y && \
    sudo apt-get install ros-jazzy-gz-ros2-control* -y
RUN sudo apt-get install ros-jazzy-moveit* -y

# Needed for Arduino coms
RUN sudo apt-get install libserial-dev -y

RUN sudo apt install python3.12-venv -y
 


