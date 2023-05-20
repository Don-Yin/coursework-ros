1. **Setup Your Environment**: Source your ROS environment.

    ```bash
    source /opt/ros/noetic/setup.bash
    ```

2. **Run `roscore`**: Start `roscore` by typing the command `roscore`. `roscore` is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have it running in order to use ROS.

3. **Start MoveIt**: If you have a MoveIt configuration package for your robot, you might need to start the MoveIt Rviz plugin, which provides the user interface for MoveIt. This is typically done by running a command similar to `roslaunch <your_robot_moveit_config>_demo.launch`.

    ```bash
    roslaunch move_robot_moveit_config demo.launch
    ```

4. **Execute Your Script**: Now, navigate to the directory of your script in a new terminal window. Make sure that your script has executable permissions by running `chmod +x script.py`. Then, run your script with the `rosrun` command by specifying the package name and script name. If you don't have a specific package for your script, you can also run it directly by using `python script.py`.

5. **Monitor Your Topics**: You can monitor your topics using the `rostopic` command. For example, `rostopic echo /target_pose` will print messages published to the `/target_pose` topic.
