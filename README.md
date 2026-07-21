# HINT: Human-Inspired Navigation Topology

## Overview

This project implements a navigation strategy based on [human spatial navigation](https://press.princeton.edu/books/hardcover/9780691171746/human-spatial-navigation). The system is implemented in the ROS2 framework and tested on the [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot.

> Check the architectural diagram [here](https://lucid.app/lucidchart/8486b168-5023-43c9-aef5-2acd9b74a0a5/edit?viewport_loc=1877%2C759%2C1332%2C708%2C0_0&invitationId=inv_f83a56f2-bcd1-4c2f-bc68-6e68dcb5b84c)!

## Motivation

Despite having an extremely poor geometric sense of the world, humans still manage to navigate through space day to day without bigger issues. Although not perfect, most people can reach their job site everyday and navigate through their houses flawlessly. That raises a question: is a high-fidelity geometric reconstruction of the world really needed for navigation?

This provocation pokes right into prohibitive 3D scanner costs, extensive mapping and re-mapping efforts, the fragility of solutions on ever changing environments and lack of semantic information integration on the navigation process.

Recently, robots are leaving controlled industrial sites and are integrating into uncertain human-centric environments. In order to imitate a navigational intelligence, methods like end-to-end VLA strategies are becoming popular, but are still very experimental, most of them lack a temporal action awareness, and require extensive datasets, energy and time to be properly trained. 

Thinking of these limitations, this project proposes a structured semantic navigation model based on how human navigational heuristics. As the "Human Spatial Navigation" book suggests, humans have a huge variety of modular and codeable navigation strategies that can be translated to a robotic system. 

## Usage
To build and deploy the project, follow these steps:

1. **Build the Docker image**:
   ```bash
   docker compose up -d
   ```

2. **Enter the container**:
   ```bash
   docker exec -it turtlebot3 bash
   ```

3. **Compile the ROS2 packages**:
   ```bash
   colcon build --symlink-install && source install/setup.sh
   ```

4. **Run the launch file**:
   ```bash
   ros2 launch hint_bringup bringup.launch.py
   ```

---