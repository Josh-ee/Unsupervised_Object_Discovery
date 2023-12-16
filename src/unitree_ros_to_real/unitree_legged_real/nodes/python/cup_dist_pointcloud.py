import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d

def pointcloud2_to_xyz(pointcloud2_msg):
    """
    Convert a ROS PointCloud2 message to an Open3D PointCloud.

    Args:
    - pointcloud2_msg (PointCloud2): A ROS PointCloud2 message.

    Returns:
    - o3d.geometry.PointCloud: An Open3D PointCloud object.
    """
    # Read points from the PointCloud2 message
    points_list = list(pc2.read_points(pointcloud2_msg, field_names=("x", "y", "z"), skip_nans=True))

    # Convert to numpy array
    points = np.array(points_list)

    # Create an Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

def display_pointcloud():
    """
    Subscribe to the ROS PointCloud2 topic and display the point cloud using Open3D.
    """
    # Initialize ROS node
    rospy.init_node('pointcloud2_viewer', anonymous=True)

    # Define a callback function to process the PointCloud2 messages
    def callback(pointcloud2_msg):
        pcd = pointcloud2_to_xyz(pointcloud2_msg)
        o3d.visualization.draw_geometries([pcd])

    # Subscribe to the PointCloud2 topic
    rospy.Subscriber('/camera/point_cloud', PointCloud2, callback)

    # Keep the node running
    rospy.spin()

# Run the display function
display_pointcloud()
