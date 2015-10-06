/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistorter.h"

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <m3api/xiApi.h>

#include "lsd_slam_viewer/PointCloudViewer.h"
#include "lsd_slam_viewer/QGLDisplay.h"
#include "lsd_slam_viewer/viewerSettings.h" //externs with viewer parameters

#include <qapplication.h>
#include <thread>         // std::thread
#include <QtConcurrent/QtConcurrent>
#include <QThread>
#include <QSettings>

#include "keypresshandler.h"
#include "consts.h"

/* DRONE INCLUDES*/
#ifdef USE_DRONE
#include <mainwindow.h>
#endif

/* Keep the webcam from locking up when you interrupt a frame capture */
volatile int quit_signal=0;
#ifdef __unix__
#include <signal.h>
extern "C" void quit_signal_handler(int signum) {
  if (quit_signal!=0) exit(0); // just exit already
  quit_signal=1;
  printf("Will quit at next camera frame (repeat to kill now)\n");
}
#endif

// Viewer for 3D reconstructed scene, uses qglviewer
PointCloudViewer *viewer;

// Views for image streams, altogether 4 image stream will be displayed
QGLDisplay *display1;
QGLDisplay *display2;
QGLDisplay *display3;
QGLDisplay *display4;
//...

#ifdef USE_DRONE
MainWindow *droneWindow;
#endif

QSettings *settings;

enum class VideoSource { Images, Camera, Drone };
static VideoSource videoSource;

class KeyPressHandler;
KeyPressHandler *kph;

bool startLSDSlam;
bool reset2;

int mainLoopCodeForQtThread();

std::string &ltrim(std::string &s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}
std::string &rtrim(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}
std::string &trim(std::string &s) {
  return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL)
    {
      return -1;
    }

  while ((dirp = readdir(dp)) != NULL) {
      std::string name = std::string(dirp->d_name);

      if(name != "." && name != "..")
        files.push_back(name);
    }
  closedir(dp);


  std::sort(files.begin(), files.end());

  if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
  for(unsigned int i=0;i<files.size();i++)
    {
      if(files[i].at(0) != '/')
        files[i] = dir + files[i];
    }

  return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
  std::ifstream f(source.c_str());

  if(f.good() && f.is_open())
    {
      while(!f.eof())
        {
          std::string l;
          std::getline(f,l);

          l = trim(l);

          if(l == "" || l[0] == '#')
            continue;

          files.push_back(l);
        }

      f.close();

      size_t sp = source.find_last_of('/');
      std::string prefix;
      if(sp == std::string::npos)
        prefix = "";
      else
        prefix = source.substr(0,sp);

      for(unsigned int i=0;i<files.size();i++)
        {
          if(files[i].at(0) != '/')
            files[i] = prefix + "/" + files[i];
        }

      return (int)files.size();
    }
  else
    {
      f.close();
      return -1;
    }
}

using namespace lsd_slam;
int main( int argc, char** argv )
{

#ifdef __unix__
  signal(SIGINT,quit_signal_handler); // listen for ctrl-C
#endif

  startLSDSlam = false;
  reset2 = false;

  QApplication application(argc, argv);
  setlocale(LC_NUMERIC,"C");

//  settings = new QSettings("../settings.ini", QSettings::IniFormat);
  settings = new QSettings("settings.ini", QSettings::IniFormat);

  qDebug() << "Paths/ImagePath: " << settings->value("Paths/ImagePath");
  qDebug() << "Paths/CameraCalibPath: " << settings->value("Paths/CameraCalibPath");
  qDebug() << "Video/Source: " << settings->value("Video/Source");
  qDebug() << "Video/CameraNumber: " << settings->value("Video/CameraNumber");

  //set video source
  QString val = settings->value(VIDEO_SOURCE_KEY, VIDEO_SOURCE_DEFAULT).toString();
  if(val == SOURCE_NAME_CAMERA){
      videoSource = VideoSource::Camera;
    }
  else if(val == SOURCE_NAME_DRONE)
    videoSource = VideoSource::Drone;
  else
    videoSource = VideoSource::Images;

  //set thresholds
  scaledDepthVarTH = settings->value(CONFIDENCE_THR_KEY, CONFIDENCE_THR_DEFAULT).toFloat();
  absDepthVarTH = scaledDepthVarTH;
  qDebug() << scaledDepthVarTH;

  // Instantiate the viewer of the scene reconstruction.
  kph = new KeyPressHandler();
  kph->setStartVar(&startLSDSlam);
  kph->setResetVar(&reset2);

  viewer = new PointCloudViewer();
  viewer->setWindowTitle("PointCloud Viewer");
  viewer->installEventFilter(kph);

  display1 = new QGLDisplay();
  display2 = new QGLDisplay();
  display3 = new QGLDisplay();
  display4 = new QGLDisplay();

  display1->setWindowTitle("Debug Depth");
  display2->setWindowTitle("Tracking Residual");
  display3->setWindowTitle("Stereo Keyframe");
  display4->setWindowTitle("Stereo Reference Frame");

  int sizew = 500;
  int sizeh = 360;

  display1->resize(sizew, sizeh);
  display1->move(0,0);
  display2->resize(sizew, sizeh);
  display2->move(0,1.1*sizeh);
  display3->resize(sizew, sizeh);
  display3->move(1.1*sizew,0);
  display4->resize(sizew, sizeh);
  display4->move(1.1*sizew,1.1*sizeh);

  viewer->resize(1.5*sizew, 1.5*sizeh);
  viewer->move(2.4*sizew, 2.4*sizeh);

  // Make the viewer window visible on screen.
  viewer->show();

  display1->show();
  display2->show();
  display3->show();
  display4->show();

  //    // Camera related stuff
  //    QCamera *camera;
  //    QCameraImageCapture *imageCapture;
  //    QCameraViewfinder *viewfinder;

  //    if (QCameraInfo::availableCameras().count()==0){
  //        printf("No cameras available\n");
  //    }

  //    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
  //    foreach (const QCameraInfo &cameraInfo, cameras) {
  //        printf("%s\n", cameraInfo.deviceName().toStdString().c_str());
  //        if (cameraInfo.deviceName() == "/dev/video1"){
  //            camera = new QCamera(cameraInfo);
  //        }
  //    }

  //    imageCapture = new QCameraImageCapture(camera);
  //    camera->setCaptureMode(QCamera::CaptureStillImage);

  //    QImageEncoderSettings settings;
  //    settings.setCodec("image/jpeg");
  //    settings.setResolution(640,480);
  //    imageCapture->setEncodingSettings(settings);

  //    //connect(imageCapture, SIGNAL(imageCaptured(int,QImage)),this, SLOT(processCapturedImage(int,QImage)));

  //    viewfinder = new QCameraViewfinder;
  //    viewfinder->setFixedSize(640,480);
  //    camera->setViewfinder(viewfinder);
  //    viewfinder->show();

  //    camera->start(); // Viewfinder frames start flowing

  //    //on half pressed shutter button
  //    camera->searchAndLock();

  //    //on shutter button pressed
  //    imageCapture->capture();

  //    //on shutter button released
  //    camera->unlock();
  //#ifdef USE_DRONE
  //    if(videoSource == VideoSource::Drone) {
  //        droneWindow = new MainWindow();
  //        droneWindow->installEventFilter(kph);
  //        droneWindow->show();
  //    } else {
  //        droneWindow = NULL;
  //    }
  //#endif

  QtConcurrent::run(mainLoopCodeForQtThread);
  //QFuture<void> future = QtConcurrent::run(mainLoopCodeForQtThread);
  //future.waitForFinished();
  //return 0;
  return application.exec();
}

int mainLoopCodeForQtThread()
{
  // get camera calibration in form of an undistorter object.
  // if no undistortion is required, the undistorter will just pass images through.
  Undistorter* undistorter = 0;

  undistorter = Undistorter::getUndistorterForFile(settings->value(CALIB_PATH_KEY, CAMERA_CALIB_PATH_DEFAULT).toString().toLatin1());

  if(undistorter == 0)
    {
      printf("need camera calibration file! (set using _calib:=FILE)\n");
      exit(0);
    }

  int w = undistorter->getOutputWidth();
  int h = undistorter->getOutputHeight();

  int w_inp = undistorter->getInputWidth();
  int h_inp = undistorter->getInputHeight();

  float fx = undistorter->getK().at<double>(0, 0);
  float fy = undistorter->getK().at<double>(1, 1);
  float cx = undistorter->getK().at<double>(2, 0);
  float cy = undistorter->getK().at<double>(2, 1);
  Sophus::Matrix3f K;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

  // make output wrapper. just set to zero if no output is required.
  //Output3DWrapper* outputWrapper = NULL;// = new ROSOutput3DWrapper(w,h);
  Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(w,h);

  // Set pointcloudviewer pointer in OutputWrapper
  outputWrapper->setViewer(viewer);
  outputWrapper->setViews(display1, display2, display3, display4);

  // make slam system
  SlamSystem* system = new SlamSystem(w, h, K, doSlam);
  system->setVisualization(outputWrapper);

  // open image files: first try to open as file.
  std::string source;
  std::vector<std::string> files;

  cv::Mat image = cv::Mat(h,w,CV_8U);
  int runningIDX=0;
  float fakeTimeStamp = 0;

  //Initialize video source
  // Use camera
  if (videoSource == VideoSource::Camera || videoSource == VideoSource::Drone){
      cv::VideoCapture webcam(settings->value(CAMERA_NUMBER_KEY, CAMERA_NUMBER_DEFAULT).toInt());
      //cv::VideoCapture webcam(0);
      qDebug() << "camera opened: " << webcam.isOpened();

      if(videoSource==VideoSource::Camera) {
          if(!webcam.isOpened())
            {
              qDebug() << "camera could not be opened!!";
              printf("Error: cannot open stream from webcam\n");
              return -1;
            }
          webcam.set(CV_CAP_PROP_FRAME_WIDTH, w_inp);
          webcam.set(CV_CAP_PROP_FRAME_HEIGHT, h_inp);
          webcam.set(CV_CAP_PROP_FPS, 30.0);
        }
      while(true)
        {

          if (!startLSDSlam){
              qDebug() << "checkpoint2 ";
              QThread::msleep(100);
              continue;
            }
          qDebug() << "frame capture";

          cv::Mat frame;
          if(videoSource == VideoSource::Camera){
              webcam >> frame;
              //cv::imshow("test", frame);
              //qDebug() << "ok: frame captured";



//
            } else {
              //@TODO add sleep or change method
              //#ifdef USE_DRONE
              //                frame = droneWindow->getImage(); //non-blocking call
              //#endif
            }
          if (quit_signal) break; // exit cleanly on interrupt

         // printf("Processing webcam/drone image!\n");

          if (frame.empty() || frame.data==NULL || (frame.channels()!=3 && frame.channels()!=4) ){
              printf("Image empty\n");
              continue;
            }

          cv::Mat imageDist;
          cv::cvtColor(frame, imageDist, CV_BGR2GRAY);
          printf ("After cvtColor...\n");
          //outputWrapper->showKeyframeDepth(frame);

          //            if(imageDist.rows != h_inp || imageDist.cols != w_inp)   {
          //                    printf("image has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",w,h,imageDist.cols, imageDist.rows);
          //                continue;
          //            }
          assert(imageDist.type() == CV_8U);

          undistorter->undistort(imageDist, image);
          assert(image.type() == CV_8U);

          if(runningIDX == 0)
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
          else
            system->trackFrame(image.data, runningIDX ,true,fakeTimeStamp);
          runningIDX++;
          fakeTimeStamp+=0.03;

          if(reset2)
            {
              qDebug() << "FULL RESET!";
              delete system;

              system = new SlamSystem(w, h, K, doSlam);
              system->setVisualization(outputWrapper);

              reset2 = false;
              runningIDX = 0;
            }

        }

    }else if(videoSource == VideoSource::Images){
      source = settings->value(IMAGES_PATH_KEY, IMAGES_PATH_DEFAULT).toString().toStdString();

      if(getdir(source, files) >= 0)
        {
          printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
        }
      else if(getFile(source, files) >= 0)
        {
          printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
        }
      else
        {
          printf("could not load file list! wrong path / file?\n");
        }

      for(unsigned int i=0;i<files.size();i++)
        {
          printf("\tProcessing image %s!\n", files[i].c_str());

          cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

          if(imageDist.rows != h_inp || imageDist.cols != w_inp)
            {
              if(imageDist.rows * imageDist.cols == 0)
                printf("failed to load image %s! skipping.\n", files[i].c_str());
              else
                printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                       files[i].c_str(),
                       w,h,imageDist.cols, imageDist.rows);
              continue;
            }
          assert(imageDist.type() == CV_8U);

          undistorter->undistort(imageDist, image);
          assert(image.type() == CV_8U);

          if(runningIDX == 0)
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
          else
            system->trackFrame(image.data, runningIDX ,true,fakeTimeStamp);
          runningIDX++;
          fakeTimeStamp+=0.03;

          if(fullResetRequested)
            {

              printf("FULL RESET!\n");
              delete system;

              system = new SlamSystem(w, h, K, doSlam);
              system->setVisualization(outputWrapper);

              fullResetRequested = false;
              runningIDX = 0;
            }
        }
    }

  system->finalize();

  delete system;
  delete undistorter;
  delete outputWrapper;
#ifdef USE_DRONE
  if(videoSource == VideoSource::Drone){
      delete droneWindow;
    }
#endif
  delete settings;

  return 0;
}
