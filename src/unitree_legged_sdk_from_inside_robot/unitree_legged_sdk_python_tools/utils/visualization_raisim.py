import os
import numpy as np
import pdb
import raisimpy as raisim

class VisualizeRaisim():

    def __init__(self):

        # Construct floor:
        # https://raisim.com/sections/HeightMap_example_png.html

        # raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
        # raisim.World.setLicenseFile("/home/ubuntu/.raisim/activation.raisim") # Not sure this actually works...
        # pdb.set_trace()

        # go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_pybullet_modif/data/go1_description/urdf/go1.urdf"
        # go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_pybullet_modif/data/go1_description/urdf/go1_no_gazebo.urdf"

        # self.go1_urdf_file = "/home/ubuntu/mounted_home/work/code_projects_WIP/unitree_mujoco/data/go1/urdf/go1.urdf" # amarco laptop, ubuntu VM machine
        self.go1_urdf_file = "/home/amarco/catkin_real_robot_ws/src/unitree_legged_sdk_from_inside_robot/assets/go1/urdf/go1.urdf" # laptop robot lab
        """
        NOTE: When running on a virtual machine, self.go1_urdf_file needs to point to the mounted volume (i.e., /home/ubuntu/mounted_home ...)
        However, when running the RaiSimUnity from macOS, the meshes won't be found because they are defined inside the URDF as a relative import and such path
        cannot be accesses outisde the virtual machine, i.e., the RaiSimUnity App won't find it.

        Solution: hardcode the path to the meshes inside the URDF, as to be found from the macOS
        """


        self.world = raisim.World()
        # world.setTimeStep(0.001) # not sure we need this...
        self.server = raisim.RaisimServer(self.world)
        ground = self.world.addGround()

        self.go1_nominal_joint_config = np.array([0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
                                            0.0136, 0.7304, -1.4505,  # FR [hip lateral, hip, knee]
                                            -0.0118, 0.7317, -1.4437, # FL [hip lateral, hip, knee]
                                            0.0105, 0.6590, -1.3903, # RR [hip lateral, hip, knee]
                                            -0.0102, 0.6563, -1.3944]) # RL [hip lateral, hip, knee]
        self.go1_nominal_joint_config[2] += 0.3 # Robot hanging

        self.go1_visual_joint_pos_curr = self.go1_nominal_joint_config.copy()
        self.go1_visual_joint_pos_des = self.go1_nominal_joint_config.copy()

        self.go1_visual_curr = self._add_visual_go1_instance(name="go1_curr")
        self.go1_visual_des  = self._add_visual_go1_instance(name="go1_des",color=[0.0,0.0,0.9,0.3])

        print("Launching Raisim server...")
        self.server.launchServer(8080)

    def _add_visual_go1_instance(self,name,color=None):

        # Visual object (without physics):
        go1_visual_curr = self.server.addVisualArticulatedSystem(name,self.go1_urdf_file) # Add a visual capsule without physics. See src/world.cpp
        go1_visual_curr.setGeneralizedCoordinate(self.go1_nominal_joint_config)
        if color is not None:
            go1_visual_curr.setColor(*color) # [R,G,B,transparency]

        return go1_visual_curr

    def update_visualization(self,joint_pos_curr,joint_pos_des):

        self.go1_visual_joint_pos_curr[7::] = joint_pos_curr
        self.go1_visual_joint_pos_des[7::] = joint_pos_des
        self.go1_visual_curr.setGeneralizedCoordinate(self.go1_visual_joint_pos_curr)
        self.go1_visual_des.setGeneralizedCoordinate(self.go1_visual_joint_pos_des)


