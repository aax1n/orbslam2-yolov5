echo "Monocular_KITTI "


cd /home/xin/catkin_ws/src/ORB_SLAM2

rosrun ORB_SLAM2 RGBD_datasets Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml
