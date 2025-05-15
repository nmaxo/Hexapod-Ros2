# PhantomX Robot Simulation in Gazebo with ROS 2 Humble

![Hexapod](https://github.com/user-attachments/assets/10c928e9-9ac2-4a77-91c2-b48b27461df9)


This repository contains the simulation package of the PhantomX(https://github.com/HumaRobotics/phantomx_gazebo) robot with new in Gazebo adopted for  ROS 2 Humble. The package includes robot description, control configurations, and Gazebo launch files.

## Prerequisites

- **ROS 2 Humble Hawksbill** (installed and configured)
- **Gazebo** (recommended version: Fortress or newer)
- **colcon** build system

## Installation and Setup

1. Clone this repository into your ROS 2 workspace:
   ```bash
   git clone https://github.com/your-repository/phantomx_simulation.git
   ```

2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Building the Project

To build all packages:
```bash
colcon build
```

For specific packages (robot description, Gazebo simulation, and control):
```bash
colcon build --packages-select phantomx_gazebo phantomx_description phantomx_control joint_state_publisher
```

## Running the Simulation

### Launch the complete simulation:
```bash
ros2 launch phantomx_gazebo phantomx_gazebo.launch.py
```


```


```

## Package Structure

- `phantomx_description`: Contains URDF files and meshes for the PhantomX robot
- `phantomx_gazebo`: Gazebo simulation launch files and configurations
- `phantomx_control`: Control configurations and launch files
- `joint_state_publisher`: Package for publishing joint states

## Troubleshooting

If you encounter any issues:
- Verify all dependencies are installed (`rosdep install`)
- Check Gazebo version compatibility
- Ensure proper sourcing of the workspace (`source install/setup.bash`)

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

