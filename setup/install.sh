#!/bin/bash
# Instala el entorno definido en ADR-001 sobre Ubuntu 24.04.
# La misma combinacion debe quedar en las tres maquinas del equipo.
set -euo pipefail

echo "==> Repositorio de ROS2"
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    sudo curl -sSL \
        https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
fi

echo "==> ROS2 Jazzy y simulacion"
sudo apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-moveit \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-yaml \
    build-essential git

echo "==> rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update

echo "==> Entorno en el shell"
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo
echo "==> Listo. Comprueba con:"
echo "    source /opt/ros/jazzy/setup.bash && ros2 doctor --report | head"
