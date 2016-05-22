# microRobotsROS
Repository of code for the micro robots for education project. Provides a ros interface for the small robots. 

mkdir -p ~/uRobot_ws/src
cd ~/uRobot_ws/src
catkin_init_workspace
git clone https://github.com/je310/microRobotsROS ~/uRobot_ws/src 
cd ~/uRobot_ws/
catkin_make 
run using roslaunch urobot runrobots.launch
