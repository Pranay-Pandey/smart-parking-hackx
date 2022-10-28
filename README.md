# smart-parking-hackx

This development is a part of the HackX- Concetto Hackathon

The problem Statement chosen is Real-Time Parking View

Parking lots public spaces are often crowded and finding a spot amidst the
chaos becomes a hectic task. To make the lives of car owners easier and to
assist them in finding an empty parking spot, design an IOT based system

This is a ROS stimulation- To run it in your own systems follow the steps

Install ROS and gazebo from - 
sudo apt-get install ros-kinetic-full-desktop

make a workspace and src folder
mkdir catkin_ws && cd catkin_ws
mkdir src

copy the files in this repo inside src and then build the catkin workspace
catkin build

now source this package before running the simulation
Run the simuation via
roslaunch park parkingLot.launch


