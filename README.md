# Unsupervised Object Discovery via Interaction

Developed an LLM-aware computer vision algorithm - dependent on physically probing the built environment - equipping a quadruped unmanned ground vehicle (UGV) for unsupervised object detection.


<div align="center">
  <img src="media/read_me.gif" alt="Description">
</div>


</br>

To view final results:
- [Project Website](https://sites.google.com/berkeley.edu/unsupervised-object-discovery/unsupervised-object-discovery-via-interaction)
- [Presenatation Deck](https://docs.google.com/presentation/d/1bJXGHLaNxGCH2Xnr3C20kdvZxDNgF2fkvTv9KjHSBMs/edit?usp=sharing)


This repo contains the forked versions of the following repos:
- https://github.com/alonrot/unitree_ros_to_real.git
- https://github.com/alonrot/unitree_legged_sdk_from_inside_robot.git

Specficially the public branch of both.
This code would not be possible without the amazing work of alonrot, and his [documentation](https://catkin-denim-4f0.notion.site/Go1-Setup-Simple-Walking-Test-0e0e9ae4ed074a53b2bb31e62ac6f73e)


Since both repos have to be in the same ROS workspace to move the robot, our team found it easier to combine into a single repo


# Setup

Before running the experiment, make sure to:
- copy the files from src/pi_files to the Raspberry Pi
- copy the files from src/nano_files to the main Jetson Nano (192.168.123.13)
