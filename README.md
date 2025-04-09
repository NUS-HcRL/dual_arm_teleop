# OPEN TEACH: Airbot Teleoperation System Integration

### Environment Setup

1. Install the required conda environment:

**Allegro Sim**:
```bash
conda env create -f env_isaac.yml
```

**Other Environments**:
```bash
conda env create -f environment.yml
```

2. After installing the dependencies, install the pipeline package using pip:
```bash
pip install -e .
```

3. Verify the installation by running the following command:
```bash
import openteach
```

### Launch Meta Quest 3

1. Use **SideQuest** to connect the Meta Quest 3 to your computer. Make sure your VR device is properly connected and developer mode is enabled.

2. Open the APK file on the Meta Quest 3:

   - **BimanualArm.apk**: For bimanual operation.
   - **SingleArmBot.apk**: For single-arm operation.

### Start Camera Sensors

1. Ensure the camera is connected before starting.
2. Modify the RealSense number in `configs/camera.yaml` and run the following command to start the server:
```bash
bash launch_server.sh
```

### Launch Airbot Robot Arm

Open Teach supports the following three Airbot control modes:

1. **Airbot Bimanual**: For bimanual operation.
2. **Airbot ROS**: For integration with ROS.
3. **Airbot**: For standard single-arm control.

Choose the appropriate mode and start by running the following commands:

- For bimanual mode:
  ```bash
  python teleop.py robot=airbot_bimanual
  ```
- For ROS mode:
  ```bash
  python teleop.py robot=airbot_ros
  ```
- For single-arm mode:
  ```bash
  python teleop.py robot=airbot
  ```


### Citation
If you use this repo in your research, please consider citing the paper as follows:
```
@misc{iyer2024open,
      title={OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation}, 
      author={Aadhithya Iyer and Zhuoran Peng and Yinlong Dai and Irmak Guzey and Siddhant Haldar and Soumith Chintala and Lerrel Pinto},
      year={2024},
      eprint={2403.07870},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}



