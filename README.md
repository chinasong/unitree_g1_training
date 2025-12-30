# Unitree G1 Training Project

This project is designed for training and development with the Unitree G1 robot platform. It includes various tools and utilities for robot control, simulation, and data processing.

## 🚀 Quick Start

### TTS Service (Text-to-Speech)
The easiest way to control G1 robot's speech:

```bash
# Start TTS service
python3 g1_tts_service.py --iface wlan0 --server

# Access web interface
# Open http://192.168.1.28:8080 in browser

# Or use Python client
python3 g1_tts_client_example.py
```

See [G1_TTS_SERVICE_README.md](G1_TTS_SERVICE_README.md) for detailed documentation.

## Project Structure

- **Core Services**:
  - `g1_tts_service.py`: TTS service with HTTP API and web interface
  - `g1_tts_client_example.py`: Python client example
  - `g1_tts_simple_test.py`: Simple test script
  - `g1_start_tts_service.sh`: Quick start script

- **Development Tools**:
  - `armdev/`: Arm control and data processing tools
  - `cameradev/`: Camera and vision development
  - `recorddata/`: Data recording and processing
  - `videodev/`: Video processing tools
  - `g1dev/`: G1 device utilities
  - `legdev/`: Leg control development

- **Utilities**:
  - `g1_remote_exec.sh`: Remote command execution tool
  - `quick_start_tts.sh`: Quick TTS service launcher

- **Documentation**:
  - `G1_TTS_SERVICE_README.md`: Complete TTS service documentation
  - `G1_STANDING_LONG_JUMP_README.md`: Standing long jump feature
  - `G1_JOINT_RANGE_LIMITS.md`: Joint range limits
  - `G1_MOTOR_INFO.md`: Motor information
  - `doc/`: Additional documentation

## Dependencies

The project relies on several external dependencies:
- Unitree SDK 2.0 (Python)
- Unitree RL Gym
- ROS (Robot Operating System)
- MuJoCo (for simulation)
- PCL (Point Cloud Library)
- Eigen
- OpenMP
- CUDA (optional, for GPU acceleration)
- Sophus
- nvbio

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

# Build fast_gicp (optional, for point cloud processing)
cd src/fast_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
# For CUDA support:
# cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_VGICP_CUDA=ON
make -j8
```

3. Build the ROS workspace:
```bash
cd catkin_ws
catkin_make
```

## Usage

### TTS Service (Recommended)
The easiest way to interact with G1 robot:

```bash
# Start HTTP server
python3 g1_tts_service.py --iface wlan0 --server

# Interactive mode
python3 g1_tts_service.py --iface wlan0 --interactive

# Say one sentence
python3 g1_tts_service.py --iface wlan0 --speak "Hello G1"
```

### Arm Control
```bash
python armdev/armdev_data.py
python armdev/g1_csv_to_lowcmd_player_with_legs.py
```

### Data Processing
```bash
# Convert CSV to MuJoCo
python armdev/g1_csv_to_mujoco.py

# Process recorded data
python recorddata/smooth_g1_csv.py
```

### Camera Development
```bash
cd cameradev
python g1_humorous_chat.py
```

### Remote Execution
```bash
# Execute commands on G1 robot
./g1_remote_exec.sh "hostname"
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