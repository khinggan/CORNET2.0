# CORNET2.0

Co-Simulation Framework for applications involving Networked Robots like Connected Autonomous Vehicles, Network of UAVs.

---

CORNET 1.0 integrates Ardupilot SITL in Gazebo and NS3 and the source code is available [here](https://github.com/srikrishna3118/CORNET.git).

CORNET 2.0 is more generic framework that can be integrated with any robotic framework that supports ROS. This code was tested on an Ubuntu 16.04 and Ubuntu 18.04 system.

For network realization, we use Mininet Wi-Fi; the dependent packages need to installed as mentioned in mininet wifi documentation. This work is submitted for review (COMSNETS 2022). If you find this code useful in your research, please consider citing:

```
@misc{acharya2021cornet,
    title={CORNET 2.0: A Co-Simulation Middleware for Robot Networks}, 
    author={Srikrishna Acharya and Bharadwaj Amrutur and Mukunda Bharatheesha and Yogesh Simmhan},
    year={2021},
    eprint={2109.06979},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```

## Installation

### mininet-WiFi + Containernet Support

**We highly recommend using Ubuntu version 16.04 or higher. Some new hostapd features might not work on Ubuntu 14.04.**

My environment is

- Ubuntu 20.04
- ROS2 Foxy

install [mininet-Wifi](https://mininet-wifi.github.io/), [containernet](https://containernet.github.io/), [docker](https://docs.docker.com/desktop/install/linux-install/)

### Run

build docker

```bash
# if the files changed in docker directory, run it. 
cd docker_container
./build.sh
```

network_config

```bash
cd CORNET2.0
sudo python3 network/network_config.py one_ap_three_stas.yaml
```

Run gzserver and spawn robots (According to [this_link](https://github.com/khinggan/comm_based_mrs_formation/pull/28))

```bash
source <comm_based_mrs_formation ws>/install/setup.bash
# run spawn 3 robots scripts
ros2 launch comm_based_mrs_formation spawnrobots.launch.py

# If you want to see the gazebo, run gzclient
gzclient
```
