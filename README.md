# Coorp

Coorp is a network supporting system designed for coordinated robotic learning.
The implementation spans multiple repos (multiple libraries and the Linux kernel), thus there are several repos for different parts of the implementation.
We describe the contents in each repo and the overall usage of Coorp here.

This repo implements the **global controller**, **BH controller**, and **preemption controller** described in the paper.

The other repos are:

* [ROS2-rcl](https://github.com/hku-systems/Coorp-ROS2-rcl)

    Implements the proxy of preemption controller in ROS2.

* [ROS2-Fast-DDS](https://github.com/hku-systems/Coorp-ROS2-Fast-DDS)

    Makes ROS2 messages go through `AC_VI` of EDCA.

* [ROS2-rcl_interfaces](https://github.com/hku-systems/Coorp-ROS2-rcl_interfaces)

    Adds some message types to support communication between preemption controller and its proxy.

* [Linux-5.4.84-rt](https://github.com/hku-systems/Coorp-Linux-5.4.84-rt)

    Implements the **virtual-preemption layer**.

# Usage

## Compile & Install Kernel

See the [README](https://github.com/hku-systems/Coorp-Linux-5.4.84-rt#readme) of the kernel repo.
You can use [MT7612U](https://www.amazon.com/Adapter-866Mbps-300Mbps-Wireless-Network/dp/B086L3D3NB/ref=sr_1_3?dchild=1&keywords=mt7612u&qid=1621998099&sr=8-3&th=1) wireless NICs, or adapt the virtual-preemption layer for other commodity WNICs.

## Compile & Install ROS2

1. Follow the [ROS2 official guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html) to install dependencies and get the original ROS2 source code.
2. Substitute `ros2_foxy/src/eProsima/Fast-DDS`, `ros2_foxy/src/ros2/rcl`, and `ros2_foxy/src/ros2/rcl_interfaces` provided by Coorp (from the above links).
3. Build and install ROS2 according to its [official guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html).

## Run

1. Adapt your application to use the `MessageStream` class provided by `bh_controller.py` to transmit bandwidth-hungry flows like parameter updates.
2. Start the `global_controller.py`.
3. Start `preemption_controller.py` on each robot.
4. Start your multi-robot applications on each robot.
