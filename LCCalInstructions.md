# LiDAR-Camera Calibration
## Setup
### Download the Workspace
To start with LiDAR-camera calibration, you'll want to pick up a copy of the lidar-camera-calibration workspace. This can be found [here](https://github.com/heethesh/lidar_camera_calibration), or, if working with the Drive PX2, a pre-modified version of the workspace can be found under the PX2's backups, which allows for skipping modifying the launch files and python scripts. 
### Modify the Targeted Topics
In each launch file and python script, the LiDAR and camera topics are referred to in a couple of places. In each instance, the LiDAR topic should be changed to `/velodyne_points` and the camera topics should be changed to whatever is appropriate for the camera and drivers you are using. In my case, that was `/gmsl_cameras/port_0/cam_0/image_raw` and `/gmsl_cameras/port_0/cam_0/camera_info`, referring to the image and camera data, respectively. These topics are easy to find in the launch files, as the files are small, but for the main calibration script, they are located on lines 508 - 510. 
For the image_color variable, the image_raw topic should be specified.
### Position the LiDAR and Camera
The LiDAR and camera must be positioned in a specific way in order to ensure this process proceeds smoothly. The devices must rest next to each other facing approximately the same direction, so that the adjustments made to the outputs of the two don't need to be too extreme to make the data overlap.
## Operation
### Overview
The operation of the LiDAR-camera calibration process consists of three steps: recording data into a bagfile, calibrating the camera off of the data, and then calibrating the LiDAR and camera in tandem, again off of the bagfile data. Camera calibration can be done through ros directly, rather than through the workspace, but that would then require that a bagfile update script be run which applies the calibration to the bagfile, as this is normally handled automatically through the workspace's camera calibration script.  
The calibration is done through a checkerboard printed onto a larger board, which a person moves around in specific patterns to provide ample data to calibrate the camera with. Then, the person backs up somewhat to provide a decent image to calibrate the LiDAR with. More info on the camera calibration can be found at https://wiki.ros.org/camera_calibration and more information on the LiDAR calibration can be found at https://github.com/heethesh/lidar_camera_calibration.
### Recording a Bagfile
__NOTE: If you already have one of the bagfiles confirmed to work with this program, you can skip this step.__  
Recording the bagfile is a fairly straight-foward process, but getting a bagfile which is adequate for the process can be fairly difficult. The process can be simplified somewhat by running the ros `camera_calibration` program while recording the bagfile to guarantee that the recorded data is good enough for camera calibration, but there is no guarantee the file will be adequate for the LiDAR portion. For this reason, it is a good idea during the recording to spend an ample amount of time at a moderate distance from the devices, so that both the LiDAR and the camera capture the entirety of the board upon which the checkerboard is printed or secured.  
To begin the process, launch the LiDAR and camera drivers you are using. With the PX2, that was the velodyne_pointcloud program and the gmsl drivers. The velodyne pointcloud could be launched from anywhere with `roslaunch velodyne_pointcloud VLP16_points.launch`, but for the gmsl drivers, the following commands had to be run, starting from the home directory.  
```
cd ros_gmsl_workspace  
source devel/setup.bash  
roslaunch gmsl_n_cameras gmsl_n_cameras_one.launch  
```
Once the LiDAR and camera are running, we can begin recording the data they are outputting. this is done through the `rosbag record` command. However, we need to record the bagfile in a specific directory in order for the calibration process to function properly. Inside the workspace, you'll want to navigate to `(workspace)/src/lidar_camera_calibration/bagfiles/`. If there is no "bagfiles" folder, than make one. Once the terminal is in the specified folder, you can begin recording the bagfile. If you want to make the process go more smoothly, though, you can also run the ros camera calibration script with the command `rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.108 image:=[YOUR_CAMERA_IMAGE] camera:=[YOUR_CAMERA]`. In the case of the PX2, the camera image is `/gmsl_cameras/port_0/cam_0/image_raw` and the camera is `/gmsl_cameras/port_0/cam_0`.  
To record the bagfile, run the command `rosbag record -O [FILENAME] [LIDAR-TOPIC] [CAMERA-IMAGE] [CAMERA-INFO]`. In my case, the specific command was `rosbag record -O calinput /velodyne_points /gmsl_cameras/port_0/cam_0/image_raw /gmsl_cameras/port_0/cam_0/camera_info`. This will begin a recording of all the data being gathered by the LiDAR and the camera. During this time, you'll want to move the checkerboard around in a variety of ways. For more info, see the ros wiki page on camera calibration I linked earlier.
### Camera Calibration
### LiDAR-Camera Calibration
## Display Results
