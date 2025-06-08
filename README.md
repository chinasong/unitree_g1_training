# Unitree G1 Training Project

This project is designed for training and development with the Unitree G1 robot platform. It includes various tools and utilities for robot control, simulation, and data processing.

## Project Structure

- `armdev/`: Contains development tools and scripts for arm control and data processing
  - `armdev_data.py`: Data processing utilities
  - `armdev_gui.py`: GUI interface for arm control
  - `g1_csv_to_lowcmd_player_with_legs.py`: CSV to low-level command converter
  - `g1_csv_to_mujoco.py`: CSV to MuJoCo simulation converter
  - `g1_23dof.xml`: MuJoCo model definition file
  - Various test and utility scripts

- `externals/`: External dependencies
  - `unitree_sdk2_python`: Unitree SDK for Python
  - `unitree_rl_gym`: Unitree reinforcement learning environment

- `catkin_ws/`: ROS workspace for robot control

- `doc/`: Documentation files

- `videodev/`: Video development tools

- `cameradev/`: Camera development tools

- `vibes/`: Additional utilities and tools

## Dependencies

The project relies on several external dependencies:
- Unitree SDK 2.0 (Python)
- Unitree RL Gym
- ROS (Robot Operating System)
- MuJoCo (for simulation)

## Getting Started

1. Clone the repository with submodules:
```bash
git clone --recursive [repository-url]
```

2. Install the required dependencies:
```bash
# Install Unitree SDK
cd externals/unitree_sdk2_python
pip install -e .

# Install Unitree RL Gym
cd ../unitree_rl_gym
pip install -e .
```

3. Build the ROS workspace:
```bash
cd catkin_ws
catkin_make
```

## Usage

### Arm Control
To use the arm control GUI:
```bash
python armdev/armdev_gui.py
```

### Data Processing
To convert CSV data to low-level commands:
```bash
python armdev/g1_csv_to_lowcmd_player_with_legs.py
```

### Simulation
To run MuJoCo simulation:
```bash
python armdev/g1_csv_to_mujoco.py
```

## Contributing

Please follow the standard git workflow:
1. Create a new branch for your feature
2. Make your changes
3. Submit a pull request

## License

[Add appropriate license information]

## Contact

[Add contact information] 