# imu_driver

To get running:
---

Dependencies:
    xsens ros driver (sudo apt-get install ros-indigo-xsens-driver [you must have the ros repo's added to apt])
    scipy and numpy libaries (sudo apt-get install python-numpy python-scipy python-matplotlib)
    
Steps:
    catkin_make
    rosrun xsens_driver mtnode.py _baudrate:=921600 _device:=/dev/seabee/imu  <--- baudrate and device specs are necesary. Otherwise, the driver won't successufully find the imu. /dev/ttyUSB0 may change, just use w.e the tty is for the imu. Additionally, /dev/ttyUSB0 may require chmod 666 /dev/ttyUSB0
    rosrun seabee_imu_driver imu_node.py
    rosrun seabee_imu_driver seabee_imu_filters.py
    
I think that's it, let me know if I'm missing anything 
