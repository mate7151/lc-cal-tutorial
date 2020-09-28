# LiDAR-Camera Calibration
## Foreword
This process is based off of the GitHub repository located [here](https://github.com/heethesh/lidar_camera_calibration).
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
Once the LiDAR and camera are running, we can begin recording the data they are outputting. this is done through the `rosbag record` command. However, we need to record the bagfile in a specific directory in order for the calibration process to function properly. Inside the workspace, you'll want to navigate to `(workspace)/src/lidar_camera_calibration/bagfiles/`. If there is no "bagfiles" folder, than make one. Once the terminal is in the specified folder, you can begin recording the bagfile. If you want to make the process go more smoothly, though, you can also run the ros camera calibration script with the command `rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.108 image:=[YOUR_CAMERA_IMAGE] camera:=[YOUR_CAMERA]`. In the case of the PX2, the camera image is `/gmsl_cameras/port_0/cam_0/image_raw` and the camera is `/gmsl_cameras/port_0/cam_0`. The recording will be complete once the calibrate button has appeared on the calibration window.    
To record the bagfile, run the command `rosbag record -O [FILENAME] [LIDAR-TOPIC] [CAMERA-IMAGE] [CAMERA-INFO]`. In my case, the specific command was `rosbag record -O calinput /velodyne_points /gmsl_cameras/port_0/cam_0/image_raw /gmsl_cameras/port_0/cam_0/camera_info`. This will begin a recording of all the data being gathered by the LiDAR and the camera. During this time, you'll want to move the checkerboard around in a variety of ways. For more info, see the ros wiki page on camera calibration I linked earlier. Once you feel the recording is complete, press Ctrl-C on the terminal running the recording script. This will save the bagfile.
### Camera Calibration
To run the camera calibration script, navigate back to the root directory of the workspace. Once there, enter the following commands. NOTE, if you did not earlier, you will need to modify the `camera_calibration.launch` file to specify the name of the bagfile you are using for calibration and specify the correct size of the board and the checkerboard. The checkerboard's size refers to the number of inside corners, rather than squares, and in my case, the board was 9x6 and the square size was 0.108.
```
source devel/setup.bash
roslaunch lidar_camera_calibration camera_calibration.launch
```
This will bring up a window which will be playing the data stored in the bagfile you recorded earlier. Assuming the recording is adequate, after a certain amount of time, a calibrate button will appear on the calibration window. Click the button. This will calibrate the camera and update the bagfile.  
If this process failed, check to make sure the launch file says image_raw instead of image_color on line 26. Additionally, check to ensure you edited the parameters in lines 4-6 to be correct.
### LiDAR-Camera Calibration
The LiDAR-camera calibration process will require two to three terminals open -- one to play the bagfile and the other to calibrate the equipment. The third terminal is optional, but can prove helpful, as it is used to run image_view to see what point the bagfile is at, so when you pause it for calibration, you know the point you paused at is condusive to the calibration process.    
The LiDAR-Camera calibration process is a manual process where the user pauses the bagfile at set points, where they then click on the four corners of the board, clockwise, starting from the top-left corner, in both the camera and the LiDAR data. When on the LiDAR data selection screen, make sure to select the pan tool at the top when moving around the data, otherwise, you might click on a data point by accident. A before and after of selecting the points can be found below. 
![Before selecting point](https://github.com/mate7151/lc-cal-tutorial/blob/master/lc-pics/BeforePickingCorners.png?raw=true)
![After selecting point](https://github.com/mate7151/lc-cal-tutorial/blob/master/lc-pics/AfterPickingCorners.png?raw=true)  
To operate the calibration program, the two main terminals will both need to be in the workspace directory and have already had `source devel/setup.bash` run on them. You'll then want to run the following commands, with the first command being run on the first terminal, the second on the second, and, if you are using a third terminal to view the bagfile data, the third on the third.
```
roslaunch lidar_camera_calibration play_rosbag.launch bagfile:=[YOUR-BAGFILE-NAME]
rosrun lidar_camera_calibration calibrate_camera_lidar.py --calibrate
rosrun image_view image_view image:=[CAMERA-IMAGE]
```
Your screen should look something like this.
![Everything running](https://github.com/mate7151/lc-cal-tutorial/blob/master/lc-pics/BeforeActivatingCalibration.png?raw=true)  
In my case, the bagfile name was calinput.bag and the camera image was /gmsl_cameras/port_0/cam_0/image_raw. Once all this is running, select the second terminal. The second terminal should look like this.
![Calibration terminal](https://github.com/mate7151/lc-cal-tutorial/blob/master/lc-pics/CalibrationTerminal.png?raw=true)  
When the recording has the checkerboard centered and a decent distance away from the camera and LiDAR, hit enter to begin the calibration process. Remember, select the top-left corner of the board and then every other corner, going clockwise, making sure to end by clicking the corner you started on. Once this is done, close the two data selection windows, WAIT FOR THE BAGFILE TO FINISH PLAYING, and repeat the process once the bagfile has looped. Eventually, once you've either collected an adequate amount of calibration data or the calibration process has crashed, the process should be complete.    
If you have trouble with the calibration program crashing on the first calibration, delete the two .npy files at `(workspace)/src/lidar_camera_calibration/calibration_data/lidar_camera_calibration/` and try again.
## Display Results
To display the calibration results, navigate to the workspace directory, run `source devel/setup.bash`, and then run `roslaunch lidar_camera_calibration display_camera_lidar_calibration.launch`. This will bring up a window showing the LiDAR data overlaid onto the camera data. If the points do not line up well, the calibration process went poorly, and should be repeated for a better calibration.