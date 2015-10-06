TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
#CONFIG -= qt

#DEFINES += USE_DRONE

CONFIG += c++11

QT       += core
QT       += gui
QT       += multimedia
QT       += multimediawidgets
QT       += opengl
QT       += xml

INCLUDEPATH += \
    /usr/local/include/eigen3 \
    /usr/include/suitesparse \
    /usr/include/boost \
    ../thirdparty/QGLViewerQT5 \
    /usr/local/include/opencv \
    /opt/ros/jade/include \
    ../src \
    ../cfg/cpp \
    ../thirdparty/Sophus

#INCLUDEPATH += ../QDroneControl

LIBS += \
    -lpthread \
    -lboost_system \
    -lboost_thread \
    -lboost_filesystem \
    -lboost_iostreams \
#    -L/usr/local/lib \                  # OpenCV 2.4.9 fails with Qt!
    -L/usr/lib/x86_64-linux-gnu \        # Preinstalled openCV 2.4.8
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_imgcodecs \     # for opencv > 3.0.0
    -lopencv_highgui \
    -lopencv_ml \
    -lopencv_video \
    -lopencv_features2d \
    -lopencv_calib3d \
    -lopencv_objdetect \
    -lopencv_videoio \     # for opencv > 3.0.0
#    -lopencv_contrib \     # for opencv < 3.0.0
#    -lopencv_legacy \     # for opencv < 3.0.0
    -lopencv_flann \
    -L/opt/ros/jade/lib \
    -lg2o_core \
    -lg2o_stuff \
    -lg2o_solver_csparse \
    -lg2o_csparse_extension \
    -lg2o_types_sim3 \
    -lg2o_types_sba \
    -lcsparse \
    -lcxsparse \
    #-L/usr/lib \
    -L../thirdparty/QGLViewerQT5 \
    -lQGLViewerQT5

#contains(DEFINES, USE_DRONE) {
#    #drone control libs
#    LIBS += \-L../build-QDroneControl-Release -lQDroneControl \
#                -lm                     \
#                -lpthread               \
#                -lavutil                \
#                -lavformat              \
#                -lavcodec               \
#                -lswscale               \
#}

SOURCES += \
    ../src/DataStructures/Frame.cpp \
    ../src/DataStructures/FrameMemory.cpp \
    ../src/DataStructures/FramePoseStruct.cpp \
    ../src/DepthEstimation/DepthMap.cpp \
    ../src/DepthEstimation/DepthMapPixelHypothesis.cpp \
    ../src/GlobalMapping/FabMap.cpp \
    ../src/GlobalMapping/g2oTypeSim3Sophus.cpp \
    ../src/GlobalMapping/KeyFrameGraph.cpp \
    ../src/GlobalMapping/TrackableKeyFrameSearch.cpp \
    ../src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp \
    ../src/IOWrapper/ROS/ROSImageStreamThread.cpp \
    ../src/IOWrapper/ROS/ROSOutput3DWrapper.cpp \
    ../src/IOWrapper/Timestamp.cpp \
    ../src/Tracking/least_squares.cpp \
    ../src/Tracking/Relocalizer.cpp \
    ../src/Tracking/SE3Tracker.cpp \
    ../src/Tracking/Sim3Tracker.cpp \
    ../src/Tracking/TrackingReference.cpp \
    ../src/util/globalFuncs.cpp \
    ../src/util/settings.cpp \
    ../src/util/SophusUtil.cpp \
    ../src/util/Undistorter.cpp \
    ../src/LiveSLAMWrapper.cpp \
    ../src/main_live_odometry.cpp \
    ../src/main_on_images.cpp \
    ../src/SlamSystem.cpp \
    ../src/lsd_slam_viewer/KeyFrameDisplay.cpp \
    ../src/lsd_slam_viewer/KeyFrameGraphDisplay.cpp \
    ../src/lsd_slam_viewer/PointCloudViewer.cpp \
    ../src/lsd_slam_viewer/viewerSettings.cpp \
    ../src/lsd_slam_viewer/QGLDisplay.cpp \
    ../src/keypresshandler.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    ../src/DataStructures/Frame.h \
    ../src/DataStructures/FrameMemory.h \
    ../src/DataStructures/FramePoseStruct.h \
    ../src/DepthEstimation/DepthMap.h \
    ../src/DepthEstimation/DepthMapPixelHypothesis.h \
    ../src/GlobalMapping/FabMap.h \
    ../src/GlobalMapping/g2oTypeSim3Sophus.h \
    ../src/GlobalMapping/KeyFrameGraph.h \
    ../src/GlobalMapping/TrackableKeyFrameSearch.h \
    ../src/IOWrapper/ROS/ROSImageStreamThread.h \
    ../src/IOWrapper/ROS/ROSOutput3DWrapper.h \
    ../src/IOWrapper/ROS/rosReconfigure.h \
    ../src/IOWrapper/ImageDisplay.h \
    ../src/IOWrapper/InputImageStream.h \
    ../src/IOWrapper/NotifyBuffer.h \
    ../src/IOWrapper/Output3DWrapper.h \
    ../src/IOWrapper/Timestamp.h \
    ../src/IOWrapper/TimestampedObject.h \
    ../src/Tracking/least_squares.h \
    ../src/Tracking/Relocalizer.h \
    ../src/Tracking/SE3Tracker.h \
    ../src/Tracking/Sim3Tracker.h \
    ../src/Tracking/TrackingReference.h \
    ../src/util/EigenCoreInclude.h \
    ../src/util/globalFuncs.h \
    ../src/util/IndexThreadReduce.h \
    ../src/util/settings.h \
    ../src/util/SophusUtil.h \
    ../src/util/Undistorter.h \
    ../src/LiveSLAMWrapper.h \
    ../src/SlamSystem.h \
    ../src/lsd_slam_viewer/keyframeGraphMsg.h \
    ../src/lsd_slam_viewer/keyframeMsg.h \
    ../src/lsd_slam_viewer/KeyFrameDisplay.h \
    ../src/lsd_slam_viewer/KeyFrameGraphDisplay.h \
    ../src/lsd_slam_viewer/PointCloudViewer.h \
    ../src/lsd_slam_viewer/viewerSettings.h \
    ../src/lsd_slam_viewer/QGLDisplay.h \
    ../src/keypresshandler.h \
    ../src/consts.h
