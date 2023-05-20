import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import json
from pathlib import Path
import nibabel as nib
from std_msgs.msg import String
import numpy as np
import ros_numpy
import std_msgs
from sensor_msgs.msg import PointCloud2


# Running roscore: It's important to note that if you are using roslaunch to launch any nodes, you don't need to start roscore separately because roslaunch will automatically start roscore if it is not already running.

rospy.init_node("visualize_node", anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arms"  # specify your robot group name
move_group = moveit_commander.MoveGroupCommander(group_name)


display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
)


def load_and_publish_volumes():
    files_dict_real = {
        "target_structure": [
            Path("data", "BrainParcellation", "r_hippo.nii.gz"),
        ],
        "critical_structure": [
            Path("data", "BrainParcellation", "ventricles.nii.gz"),
            Path("data", "BrainParcellation", "vessels.nii.gz"),
        ],
        "cortex": [
            Path("data", "BrainParcellation", "cortex.nii.gz"),
        ],
    }

    for structure, files in files_dict_real.items():
        for file in files:
            volume = nib.load(str(file.resolve()))
            data = volume.get_fdata()

            # Convert volume to point cloud
            x, y, z = np.where(data != 0)
            pc_array = np.zeros(
                len(x),
                dtype=[
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                ],
            )
            pc_array["x"] = x
            pc_array["y"] = y
            pc_array["z"] = z

            # Define the header
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = structure  # Using the structure name as the frame id

            # Convert Numpy array to PointCloud2
            pc_msg = ros_numpy.msgify(PointCloud2, pc_array, stamp=header.stamp, frame_id=header.frame_id)

            # Publish PointCloud2
            pub = rospy.Publisher("/" + structure + "_point_cloud", PointCloud2, queue_size=1000000)
            r = rospy.Rate(0.1)  # 0.1 Hz, adjust the rate as needed
            while not rospy.is_shutdown():
                pub.publish(pc_msg)
                r.sleep()


load_and_publish_volumes()


def callback(data):
    # Parse the incoming data and extract the required position
    position = json.loads(data.data)

    # Update the robot's position
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = position["x"]
    pose_target.position.y = position["y"]
    pose_target.position.z = position["z"]

    move_group.set_pose_target(pose_target)

    try:
        # Plan the trajectory
        plan = move_group.plan()

        # Execute the trajectory
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    except moveit_commander.MoveItCommanderException as e:
        rospy.logerr("Failed to move the robot to the target pose: {}".format(e))


rospy.Subscriber("target_pose", String, callback)

rospy.spin()
