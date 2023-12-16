# Go1 - Interface - Quadruped Robot
_How to interface with Go1 from a laptop?_

Info collected from `Unitree Robotics-legged_sdk.pptx`, `software_manual_20211201.pdf` and `unitree_legged_sdk_manual_20211204.pdf`

### Connect through WiFi (5G)
1. Connect to the WiFi `Unitree_Go197040A`
 	* pass: `00000000` (8 zeroes)
2. Connect to the robot’s onboard computer (raspberry pi):
	* `ssh pi@192.168.123.161`
	* pass: `123`
4. Connect to the nano controllers that then connect to the cameras, etc:
	* `ssh unitree@192.168.123.13`
	* `ssh unitree@192.168.123.14`
	* `ssh unitree@192.168.123.15`
	* Pass: `123`


### Connect through ethernet cable
1. Start the robot
2. When it stands up, use the gamepad controller (turn it on first) and press
	* L2+B: The robot will sit down on the floor
	* L1+L2+Start: Whatever program is running on the robot will stop and the robot won’t walk, run or react to the environment. However, the still  listens to the remote gamepad
3. Connect the ethernet cable between the laptop and the robot
	1. Connect the cable
	2. Turn off the Wifi on the laptop
	3. Make sure that ubuntu isn’t trying to automatically connect via wired connection (check the connection settings on the settings bar at the top right corner of the screen). If it is, click on disconnect.
4. Then, on the laptop, run the instructions from `unitree_legged_sdk_manual_20211204.pdf`, page 5, replacing eth0 with the name of the ethernet port, which can be seen doing `ifconfig`. In my case it’s `enp0s31f6`
```bash
sudo ifconfig enp0s31f6 down # eth0 is your PC Ethernet port
sudo ifconfig enp0s31f6 192.168.123.162/24
sudo ifconfig enp0s31f6 up
```
These instructions basically assign the IP `192.168.123.162` to the ethernet cable and should enable the connection between the laptop and the raspberryPI (RPI) of the robot. Then, to the eyes of the robot’s RPI, the laptop’s IP address is `192.168.123.162`, while to the eyes of the laptop, the RPI’s address `192.168.123.161`.
4. Do `ping 192.168.123.161` (which is the IP address of the RPI inside the robot). If the ping works, the connection laptop <-> RPI works. Also, double check that the IP of the ethernet cable has changed by doing `ifconfig`
5. ssh into the robot by doing `ssh pi@192.168.123.161` (password: 123)
6. Follow the instructions on the youtube video [Controlling Unitree Robots with Unitree_legged_sdk - YouTube](https://youtu.be/wXFQqrSywTU?t=453). We have created a script inside the raspberry pi at `/home/pi/wireless.sh` that executes some of te commands from the video (but not all, so watch the video). In summary, what we need to do int he RPI is:
	1. `sudo vi /etc/sysctl.conf` and uncomment the line `net.ipv4.ip_forward=1`
	2. Then, execute the script as
```bash
cd ~
./wireless.sh
```
7. On the laptop, run the line `sudo route add default gw 192.168.123.161`
	1. **Important**: Once we finish working on the robot and wanna connect to the Eduroam/CalVisitor Wifi from the laptop PC, it won’t work because the above line will make it impossible. To solve this, run `sudo route del default gw 192.168.123.161`
8. Now, passing data between the robot and the laptop using the UDP-based library provided by Unitree should be possible 




### ROS
* Download [GitHub - unitreerobotics/unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)
	* Follow instructions, but the compilation will fail.
	* As explained, clone the dependency `unitree_legged-sdk` to `~/catkin_ws/src`, but rename it to `unitree_legged-sdk-master`. Otherwise, the CMakeLists.txt of `unitree_legged_real` won’t find the dependency
	* Do `catkin_make`, then `catkin_make install` (not sure if the last is needed)
	* Source the environment: `source ~/catkin_ws/devel/setup.bash`


### Install ROS in mac
[melodic/Installation/macOS/Homebrew/Source - ROS Wiki](http://wiki.ros.org/melodic/Installation/macOS/Homebrew/Source)
* Many problems, mostly involving the correct path to python and pip
* tried anaconda environment, `ros`
* pip cannot be found

### WIP
* Simple send/receive UDP test doesn’t really work
	* The instructions say to connect via ethernet, I am connecting via WiFi
	* Try via ethernet
	* Try via WiFi and figure out the correct IP addresses
	* Maybe pull back the code from Facebook (and presentation, very detailed) -> hard drives
* Do we need ROS?
* Gazebo or Pybullet?
* If pybullet, how do I use the given URDF?


### Install Python bindings for unitree_legged_sdk
* Note: Doesn’t work on mac. This installation requires linking against `motion_imitation/third_party/unitree_legged_sdk/lib/libunitree_legged_sdk_amd64.so`, which is unfortunately not compiled for mac. However, it links fine on linux
```bash
cd </path/to/wherever>
git clone https://github.com/erwincoumans/motion_imitation.git
```
* Now, follow the instructions at the README.md inside the repo, under `Running MPC on the real A1 robot`. The compilation will generate a library at `motion_imitation/third_party/unitree_legged_sdk/build/robot_iterface.XXX.so`. That library is essentially an entire python module, that can be included in our scripts via `import robot_interface`.  In the `README.md` it is suggested to copy-paste the library wherever is convenient for us, but I’d prefer to leave it where it is and inform Python about where such library is located. As indicated here [Where does Python look for modules? — Functional MRI methods](https://bic-berkeley.github.io/psych-214-fall-2016/sys_path.html), Python will look inside the environment variable PYTHONPATH. So, we can simply add the line `export PYTHONPATH=$PYTHONPATH:/Users/alonrot/work/code_projects_WIP/motion_imitation/third_party/unitree_legged_sdk/build` to out ~_.zshrc (or ~_.bashrc in Ubuntu). Then, the line
```Python
import robot_interface
```
should work without problems in any script located anywhere.
* Important: If we compiled the aforementioned Python bindings using a particular anaconda environment, the same environment will need to be used to import the library. In other words, if we compile the library using the systemwide Python, and then we initialise a condo environment, and then we do `import robot_interface`, the module won’t be found (for some reason)


### Solutions
* Email to unitree
	* IP configuration
	* Rope -> how to hang it
* Pile of papers
	* PILCO -> 
	* Other papers from Deisenroth
	* Send Claire whatever paper is most useful about forward rollout prediction model



#postdoc/research_prep
