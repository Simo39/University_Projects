Our pakage requires to run the following commands, after the catkin build, in the following order in different terminals:


1) Launch the simulation

    roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
    roslaunch tiago_iaslab_simulation navigation.launch
    roslaunch tiago_iaslab_simulation apriltag.launch

Wait for the Tiago arm to close (otherwise it will crash against the wall)


2) Launch servers both from assignment_1 and assignment_2

    roslaunch assignment_1 start_servers.launch
    roslaunch assignment_2 start_servers.launch
    
OR
    
    roslaunch assignment_2 start_services.launch
    roslaunch assignment_2 start_actions.launch
    
NOTE:

start_services.launch and start_servers.launch of assignment_2 package can accepted an argument called: "poses_file"

This argument accept only file that are presents inside the directory "poses", that can be found in the same main folder of the assignment_2 package.    

By default they look for a file named "our_poses.txt" inside the folder poses (that is located in the assignment_2 package);
this file contains some arm and head poses and some waypoints (robot poses) which are necessary to let the overall program run.

These poses has been calculated with the enviroment proposed for our simulation.

In case the enviroment changes, it is possible to set new poses by changing the numbers in the already present file or
by creating a new file that present the same pattern of "our_poses.txt" file.

Thus, the best option (and the suggested one) is to copy the already existing file, rename it (e.g "new_poses.txt") and change the numerical values inside it.

REMEMBER, the file must be inside the directory "poses" of the assignment_2 package.
After this, you just need to pass the file to the file launch as argument to "poses_file".

Example:

    roslaunch assignment_2 start_servers.launch poses_file:="new_poses.txt"

WARNING:

Changing something different than just numerical values; adding more poses; deleting some poses or any kind of change that dosen't respect the same pattern file
present in the "our_poses.txt" file, may lead the program to unexpected behaviours or errors.


3) Launch human_node

    rosrun tiago_iaslab_simulation human_node


4) Launch human_client from assignment_2

    rosrun assignment_2 human_client
    
NOTE: point 3 and 4 can be launched toghether with the follwing command:

    roslaunch assignment_2 start_simulation.launch

REMEMBER: before starting the actual simulation (point 3 and 4), wait the robot to set its arm to its default position.


VIDEO and REPORT available here: 

https://unipdit-my.sharepoint.com/:f:/g/personal/anna_polato_2_studenti_unipd_it/EmukS6Pj1EFFnr93iOBFPuYBBGMsX72xb9Dt2kxnRKVNGA?e=qqtVXI