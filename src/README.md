# Contents

This repository contains essential ROS 2 packages to assist with the course on autonomous mobile robots in a simulated Gazebo environment. The packages provide the core functionalities to simulate a Differential Robot, a Toyota Prius, and the Quanser QCar. **This repository also includes packages created during the course of "Smart Mobility".**

-----

## 🚀 Getting Started

### Prerequisites

To use these packages, you must have **ROS 2 Humble** or **Jazzy** installed on your system.

### Dependencies

```bash
sudo apt update &&
sudo apt install -y ros-${ROS_DISTRO}-ros2-control \
                    ros-${ROS_DISTRO}-ros2-controllers \
                    ros-${ROS_DISTRO}-gz-ros2-control \
                    ros-${ROS_DISTRO}-ros-gz \
                    ros-${ROS_DISTRO}-ros-gz-bridge \
                    ros-${ROS_DISTRO}-joint-state-publisher \
                    ros-${ROS_DISTRO}-robot-state-publisher \
                    ros-${ROS_DISTRO}-xacro \
                    ros-${ROS_DISTRO}-joy
```


### Compilation

Follow these steps to clone and compile the packages in your ROS 2 workspace:

1.  Navigate to your ROS 2 workspace's source directory (e.g., `~/ros2_ws/src`).
2.  Clone this repository:
    ```bash
    git clone https://github.com/dsosa114/movilidad_inteligente.git
    ```
3.  Navigate back to the root of your workspace:
    ```bash
    cd ~/ros2_ws/
    ```
4.  Build the packages using `colcon`:
    ```bash
    colcon build
    ```
    This command will build only the necessary packages, ensuring a quick and clean compilation. If the build is successful, you are ready to use the helper nodes.
