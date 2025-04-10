FOLDER=/home/clp/workspace/lias_gopro/data
xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$FOLDER:/data" kalibr

source /opt/ros/noetic/setup.bash 
source /catkin_ws/devel/setup.bash