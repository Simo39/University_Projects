Our pakage requires to run the following commands, after the catkin build, in the following order in different terminals:

1) Launch the simulation

    roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full
    roslaunch tiago_iaslab_simulation navigation.launch

Wait for the Tiago arm to close (otherwise it will crash against the wall)

2) Launch servers

    roslaunch assignment_1 start_servers.launch

3) Launch client and pass poseB coordinates (geometry_msgs/Pose)

    rosrun assignment_1 client <position.x> <position.y> <position.z> <orientation.x> <orientation.y> <orientation.z>  <orientation.w>

for example:

    rosrun assignment_1 client 11.057 0.162 0.0 0.0 0.0 -0.569 0.822

NOTE: A list of tested poses can be found in the file "input_tested_poses.txt".

VIDEO and REPORT available here: 

https://unipdit-my.sharepoint.com/:f:/g/personal/anna_polato_2_studenti_unipd_it/EnNX-lhvtzROuHQheN_8GFkBd1QZvCzEemTrgk2C_sAxZA?e=8a8qqd

NOTE: inside OneDrive folder it is uploaded another version of server_corridor.cpp that uses waitForMessage() function instead of a callBack to read /scan topic. We think this last version is more correct with respect to the one with waitForMessage() but we found out that waitForMessage() version runs quicker. 

