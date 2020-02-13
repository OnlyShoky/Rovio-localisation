# Rovio-imu-camera
Rovio localisation using a camera ( bluefox MLC 200w G -UTW 7111) and IMU (mpu6050). The code includes the hardware synchronization and the calibration made for this two components

Bag test with the camera and imu calibrated and synchronized :  \
https://drive.google.com/file/d/1U5V8I_a4C22aDRN2bvX3oa4yei__YcHq/view?usp=sharing  \

Instructions pour l'instalation( Il faut ubuntu 16.04 et la version kinetic de ros :  \

cd catkin_ws/src \
catkin build \
catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON -DROVIO_NCAM=1 \

Instruction pour l'utilisation:  \
% In one terminal %  \
source devel/setup.bash \
roslaunch rovio rovio_node.launch \

%In another terminal%  \
roslaunch bluefox2 single_node.launch device:=25001581  \

Ligne d'instalations si erreur de catkin build :  \
(Serial) sudo apt-get install ros-kinetic-serial




Liens utilisé pour ce projet :  \
https://github.com/ethz-asl/rovio  \
http://grauonline.de/wordpress/?page_id=1951  \
https://github.com/ethz-asl/kindr  \


Logiciel utilisé pour la calibration et l'exportation en fichier yaml pour rovio  \
https://github.com/ethz-asl/kalibr  \
