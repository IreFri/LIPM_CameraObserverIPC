# Re-organized LIPM Walking Controller for Joystick Walking

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![CI](https://github.com/jrl-umi3218/lipm_walking_controller/workflows/CI/badge.svg?branch=topic/ci)](https://github.com/jrl-umi3218/lipm_walking_controller/actions?query=workflow%3A%22CI%22)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](http://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/index.html)

[![demo video](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2019-17-45.png)](https://youtu.be/XoXDZBgbamc)


Video link is [here](https://youtu.be/XoXDZBgbamc)

This version of LIPM walking controller is mainly developed by [Arnaud Tanguy](https://github.com/arntanguy).

He fixed the "double stepping" problem, communication delay between several PCs for the experiment, and so on.


And then, [Masato Tsuru](https://github.com/TsuruMasato) added an interface for PS4 controller, Oculus controller, and RViz gui menu.

If you want to try walking HRP4CR with PS4 Joystick, this version must be the easiest way.


___
## Required environments

Required components and libraries :
* Ubuntu 18.04 LTS
* ROS melodic (desktop full)
* mc_rtc, the latest version (2021/1/18) https://github.com/jrl-umi3218/mc_rtc
* openrtp environment (private repository)
* HRP4CR robot model and its mc_rtc support (private repository)
* Online Footstep Planner, the latest version (2021/1/18) https://github.com/isri-aist/OnlineFootstepPlanner
* PS4 controller and USB cable

__Basically, drcutl's environment setup script and mc_rtc installation script automatically set up most of those libraries.__

__Only "OnlineFootstepPlanner" is not automatically installed by those scripts.__

About the installation of OnlineFootstepPlanner, please visit [its page](https://github.com/isri-aist/OnlineFootstepPlanner)

___
## How to build

#### 1. download this repository:

``$ git clone git@github.com:TsuruMasato/lipm_walking_controller.git `` (default branch "rebase_stabilizer_ana")


#### 2. go into the repository and make "build" directory:

``$ mkdir build``


#### 3. go into the build directory and run ccmake :

``$ ccmake ..``

Turn on **AUTOLOAD_ExternalFootstepPlannerPlugin** option.

And also, don't forget to set **CMake Install prefix** to /home/*your_name*/openrtp.


#### 4. build this controller, and install it :

``$ make -j8``

``$ make install``


___
## How to run


First, please switch the mc_controller in your mc\_rtc configuration as doing below :

``$ nano ~/.config/mc_rtc/mc_rtc.yaml``

```yaml
MainRobot: HRP4CR
Timestep: 0.002
Enabled: LIPMWalking
```


### You need at least these 4 terminal windows.

* ROS core
* Choreonoid simulation
* RViz as mc_rtc control interface
* Online Footstep Planner


***

### process


#### 1. start ROS core in 1th terminal :

``$ roscore ``


#### 2. Please go to the HRP4CR directory in openrtp system in 2nd terminal :

``$ cd ~/openrtp/share/hrpsys/samples/HRP4CR``


#### 3. Start Choreonoid simulation in 2nd terminal :

``$ choreonoid sim_mc_openrtp_bush.cnoid ``

and please click the starting button in Choreonoid.

Now, the LIPM walking controller is running.
The HRP4CR robot model will keep standing.



_(if the robot fails on the ground, `$ clear-omninames.sh` sometimes solve it.)_

_(Even clear-omninames.sh cannot help you, your hrpsys-humanoid is too old. please update all libraries with drcutl script.)_


![standing HRP4CR](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-28-38.png)



#### 4. Start RViz in 3rd terminal :

`` $ roslaunch mc_rtc_ticker display.launch ``

![RViz panel](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-28-46.png)


#### 5. Plug your PS4 controller to a USB port, and start Online Footstep Planner in 4th terminal :

`` $ roslaunch online_footstep_planner run_demo_in_sim_rebase.launch ``

Joystick connection node and Online Footstep Planner node start.

If you can see the yellow warning message "received a map" like below-right side of the picture, it's success!!


![terimal_message](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-29-11.png)


___

___

## GUI menu

After starting the whole system, you have to select some GUI options.

#### 1. Go to "Walking" tab in the control panel in RViz


![Walking tab in RViz](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-40-29.png)


#### 2. Click "start standing" button.

_The stabilizer task becomes enable, and HRP4CR starts balancing._


#### 3. Set Footstep plan menu to "external"

#### 4. Change Target type to "PS4 Controller"

![Final state of RViz panel](https://github.com/TsuruMasato/lipm_walking_controller/blob/rebase_stabilizer_ana/image/Screenshot%20from%202022-01-18%2018-41-16.png)


#### 5. Now you can walk the robot via Joysticks!!


##### Left Joystick leads the robot walking foward/backward/left/right, and Right Joystick makes him turn.

___

___


## SoftFootState

The name of the controller is `LIPMWalkingSoftFoot` and the project name is `lipm_walking_controller_softfoot` to avoid collision with the default lipm walking.

### How to clone ?
```
git clone -b topic/SoftFootState git@github.com:IreFri/lipm_walking_controller.git --recursive lipm_walking_controller_softfoot
```
It'll clone directly into the `topic/SoftFootState` branch.

### Dependencies
* mc_mujoco
* copra
* TrajectoryCollection

### Installing `TrajectoryCollection`
You need to do the following
```
cd ~/path/to/ws_bwc/src/isri-aist/TrajectoryCollection
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/path/to/mc_rtc_install
make -j6 && make install
```

### Installing `copra
```
git clone https://github.com/jrl-umi3218/copra.git --recursive
cd copra
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/path/to/mc_rtc_install
make -j6 && make install
```

### Installing `LIPMWalkingSoftFoot`
```
source ~/path/to/ws_bwc/devel/setup.[bash or zsh] # This is needed in order to have access to `variable_stiffness` ros package
cd ~/path/to/lipm_walking_controller_softfoot
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/path/to/mc_rtc_install
make -j6 && make install
```

### How to configure `SoftFootState`?
Everytime you modify a .yaml, you need to re-install the controller i.e to run `make install`.
In `LIPMWalkingSoftFoot.in.yaml` you have:
1. The default config for swing trajectory from `BaselineWalkingController`:
```
SwingTraj:
  CubicSplineSimple:
    withdrawDurationRatio: 0.25
    withdrawOffset: [0, 0, 0.015] # [m]
    approachDurationRatio: 0.25
    approachOffset: [0, 0, 0.015] # [m]
    swingOffset: [0, 0, 0.05] # [m]
  LandingSearch:
    withdrawDurationRatio: 0.2
    withdrawOffset: [0, 0, 0.04] # [m]
    preApproachDurationRatio: 0.25
    approachDurationRatio: 0.2
    approachOffset: [0, 0, 0.04] # [m]
```
2. Inside a robot definition within `robot_models`, you can define the default swing trajectory you want to use. The parameters here will override the one in `SwingTraj` defined above.
```
SwingTraj:
  type: LandingSearch
  withdrawDurationRatio: 0.2
  withdrawOffset: [0, 0, 0.04] # [m]
  preApproachDurationRatio: 0.25
  approachDurationRatio: 0.2
  approachOffset: [0, 0, 0.04] # [m]
```
or
```
SwingTraj:
  type: CubicSplineSimple
  withdrawDurationRatio: 0.25
  withdrawOffset: [0, 0, 0.015] # [m]
  approachDurationRatio: 0.25
  approachOffset: [0, 0, 0.015] # [m]
  swingOffset: [0, 0, 0.05] # [m]
```
3. Inside `ObserverPipelines` you now have (the acquisition is roughly 10Hz which seems to be the maximum based on documentation):
```
- type: RangeSensor
      name: LeftRangeSensor
      gui: true
      config:
        range_sensor: "LeftFootRangeSensor"
        serial_port: "/dev/ttyUSB0"
- type: RangeSensor
  name: RightRangeSensor
  gui: true
  config:
    range_sensor: "RightFootRangeSensor"
    serial_port: "/dev/ttyUSB1"
```

To configure `SoftFootState` you can edit `src/states/etc/states.yaml`:
```
LIPMWalking::SoftFoot::Configured:
  base: LIPMWalking::SoftFoot
  with_variable_stiffness: false
  with_ankle_rotation: true
  with_foot_adjustment: false
  nr_phalanxes: 1
```
In any case, `with_variable_stiffness`, `with_ankle_rotation` and `with_foot_adjustment` can be configured by the GUI too.

Lastly, you need to update `~/.config/mc_rtc.yaml`, the controller name is `LIPMWalkingSoftFoot`

### How to use?
1. In the GUI, after clicking on `Walking/Main/Start Standing`, you have now access to `Walking/Swing` where you can select the desired swing trajectory type and configure it.
2. If you want to use the foot target pose update computed by `SoftFootState`, you need to use `LandingSearch` or `CubicSplineSimple`.
As a note: `CubicSplineSimple` seems slightly better, it's recommended to use it.
3. If you want to increase the distance in-between the foot for the walk, you need to do it via the GUI `Walking/Footstep/Extra step width [m]`
4. You can see the `RangeSensor` device in `ObserverPipelines/[Left/Right]RangeSensor`
5. You can also see the range sensor distance measurements in `SoftFoot`

### How to know where the sensor is in `/dev/`?
You can run `dmesg` and you should find something like:
```
[ 7877.541669] usb 1-4: new full-speed USB device number 11 using xhci_hcd
[ 7877.691333] usb 1-4: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64
[ 7877.691348] usb 1-4: New USB device strings: Mfr=0, Product=2, SerialNumber=0
[ 7877.691354] usb 1-4: Product: USB Serial
[ 7877.692892] ch341 1-4:1.0: ch341-uart converter detected
[ 7877.693636] usb 1-4: ch341-uart converter now attached to ttyUSB0
```
Here the intersting part is `ttyUSB0`.

### How to debug a `rtcd = 0`?
Before to try on the robot, the simple test is to do the following
```
cd ~/path/to/catkin_ws/devel/lib/mc_rtc_ticker
gdb --args mc_rtc_ticker
r (then press enter)
```
If it crashes, you can do the following:
```
bt
```
Then it'll show something like this
```
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
#1  0x00007ffff6c61859 in __GI_abort () at abort.c:79
#2  0x00007ffff7039911 in  () at /lib/x86_64-linux-gnu/libstdc++.so.6
#3  0x00007ffff704538c in  () at /lib/x86_64-linux-gnu/libstdc++.so.6
#4  0x00007ffff70453f7 in  () at /lib/x86_64-linux-gnu/libstdc++.so.6
#5  0x00007ffff70456a9 in  () at /lib/x86_64-linux-gnu/libstdc++.so.6
#6  0x00007fffdc03aed2 in void mc_rtc::log::error_and_throw<std::runtime_error, char const (&) [24], std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&>(char const (&) [24], std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&) () at /path/to/install/lib/mc_controller/fsm/states/Parallel.so
#7  0x00007fffdc020bf8 in mc_control::fsm::ParallelState::start(mc_control::fsm::Controller&) () at /path/to/install/lib/mc_controller/fsm/states/Parallel.so
#8  0x00007ffff405c10c in mc_control::fsm::State::start_(mc_control::fsm::Controller&) () at /path/to/install/lib/libmc_control_fsm.so.2
#9  0x00007ffff4067655 in mc_control::fsm::StateFactory::create(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, mc_control::fsm::Controller&, bool, mc_rtc::Configuration const&) () at /path/to/install/lib/libmc_control_fsm.so.2
#10 0x00007ffff406782e in mc_control::fsm::StateFactory::create(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, mc_control::fsm::Controller&) ()
    at /path/to/install/lib/libmc_control_fsm.so.2
#11 0x00007ffff4054a60 in mc_control::fsm::Executor::next(mc_control::fsm::Controller&) () at /path/to/install/lib/libmc_control_fsm.so.2
#12 0x00007ffff4055d64 in mc_control::fsm::Executor::run(mc_control::fsm::Controller&, bool) () at /path/to/install/lib/libmc_control_fsm.so.2
#13 0x00007ffff40319c4 in mc_control::fsm::Controller::run(mc_solver::FeedbackType) () at /path/to/install/lib/libmc_control_fsm.so.2
#14 0x00007ffff7e46d61 in mc_control::MCGlobalController::run() () at /path/to/install/lib/libmc_control.so.2
#15 0x0000555555615b0e in mc_mujoco::MjSimImpl::controlStep() ()
#16 0x00005555556215af in mc_mujoco::MjSimImpl::stepSimulation() ()
#17 0x00005555555eeae8 in simulate(mc_mujoco::MjSim&) ()
#18 0x00007ffff7071de4 in  () at /lib/x86_64-linux-gnu/libstdc++.so.6
#19 0x00007ffff73c6609 in start_thread (arg=<optimized out>) at pthread_create.c:477
#20 0x00007ffff6d5e133 in clone () at ../sysdeps/unix/sysv/linux/x86_64/clone.S:95

```
Then from the `#0` (it's called a frame), read and go down until you find something related to `mc_rtc`. In this example, it would be:
```
#6  0x00007fffdc03aed2 in void mc_rtc::log::error_and_throw<std::runtime_error, char const (&) [24], std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&>(char const (&) [24], std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&) () at /path/to/install/lib/mc_controller/fsm/states/Parallel.so
```
and the beginning is the name of the method/function where it crashed, here it is
```void mc_rtc::log::error_and_throw```.
From here, you can go to check the method/function and try to understand what happened !

As a note (this is a bad example, but the only error stack I have), `mc_rtc::log::error_and_throw` will `throw` (crash) the controller and it is intended.

### How to debug a state that can not be loaded?
```
[error] Some states could not be loaded as their base is not available, check for typos or cycles
[warning] - LIPMWalking::SoftFoot::Configured (base: LIPMWalking::SoftFoot)
```
The easiest way is to do:
```
locate NameOfState.so
ldd -d /path/to/NameOfState.so
```
then if you see something like:
```
undefined symbol:
```
then something is wrong in the `targeted_link_libraries` in the `CMakeLists.txt` in `src/states/`.
Otherwise, call for help !
