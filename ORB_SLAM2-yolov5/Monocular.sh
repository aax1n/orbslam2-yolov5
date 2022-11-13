echo "Monocular_KITTI "


cd /home/xin/catkin_ws/src/ORB_SLAM2

./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI00-02.yaml ~/datasets/kitti/kitti_odometry_gray/00
