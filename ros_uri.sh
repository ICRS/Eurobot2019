export ROS_MASTER_URI=http://$(hostname -I | cut -f1 -d ' '):11311
export ROS_IP=$(hostname -I | cut -f1 -d ' ')
