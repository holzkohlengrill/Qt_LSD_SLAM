#ifndef CONSTS
#define CONSTS


#define IMAGES_PATH_KEY     "Paths/ImagePath"
#define CALIB_PATH_KEY      "Paths/CameraCalibPath"

#define VIDEO_SOURCE_KEY    "Video/Source"
#define CAMERA_NUMBER_KEY   "Video/CameraNumber"

#define CONFIDENCE_THR_KEY  "Slam/DepthConfidenceThr"
#define CONFIDENCE_THR_DEFAULT "0.5"


//#define CAMERA_CALIB_PATH "/home/adam/dokt_ws/LSD_machine_small/cameraCalibration.cfg"
//#define CAMERA_CALIB_PATH "/home/adam/dokt_ws/camera_logitech/logitech640.cfg"
//#define IMAGES_PATH "/home/adam/dokt_ws/LSD_machine_small/images"

#define CAMERA_CALIB_PATH_DEFAULT   "/home/blazej/datasets/droneCalib.cfg"
#define IMAGES_PATH_DEFAULT         "/home/blazej/datasets/drone_1"

#define SOURCE_NAME_CAMERA "Camera"
#define SOURCE_NAME_IMAGES "Images"
#define SOURCE_NAME_DRONE  "Drone"

#define VIDEO_SOURCE_DEFAULT SOURCE_NAME_IMAGES
#define CAMERA_NUMBER_DEFAULT "0"

#endif // CONSTS

