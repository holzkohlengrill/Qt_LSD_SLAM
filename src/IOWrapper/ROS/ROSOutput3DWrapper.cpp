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

#include "ROSOutput3DWrapper.h"
#include "util/SophusUtil.h"
//#include <ros/ros.h>
#include "util/settings.h"

#include "std_msgs/Float32MultiArray.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"
#include "lsd_slam_viewer/keyframeMsg.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <QDebug>	//testing

namespace lsd_slam
{


  ROSOutput3DWrapper::ROSOutput3DWrapper(int width, int height)
  {
    this->width = width;
    this->height = height;
    /*
        liveframe_channel = nh_.resolveName("lsd_slam/liveframes");
        liveframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(liveframe_channel,1);

        keyframe_channel = nh_.resolveName("lsd_slam/keyframes");
        keyframe_publisher = nh_.advertise<lsd_slam_viewer::keyframeMsg>(keyframe_channel,1);

        graph_channel = nh_.resolveName("lsd_slam/graph");
        graph_publisher = nh_.advertise<lsd_slam_viewer::keyframeGraphMsg>(graph_channel,1);

        debugInfo_channel = nh_.resolveName("lsd_slam/debug");
        debugInfo_publisher = nh_.advertise<std_msgs::Float32MultiArray>(debugInfo_channel,1);

        pose_channel = nh_.resolveName("lsd_slam/pose");
    pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel,1);*/

    publishLvl=0;
  }

  ROSOutput3DWrapper::~ROSOutput3DWrapper()
  {
  }

  QImage Mat2QImage(cv::Mat const& src)
  {
    if (src.empty()||src.data==NULL||(src.channels()!=3&&src.channels()!=4)){
        printf("Null image passed to drawing...\n");
        return QImage();
      }

	//printf("Mat2QIm start\n");
    cv::Mat temp; // make the same cv::Mat
    cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
  }

  cv::Mat QImage2Mat(QImage const& src)
  {
    cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
    cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
    cvtColor(tmp, result,CV_BGR2RGB);
    return result;
  }

  void ROSOutput3DWrapper::showKeyframeDepth(const cv::Mat& image){

//    //testing
//    //
//    cv::Mat ch1, ch2, ch3, test;
//    // "channels" is a vector of 3 Mat arrays:
//    std::vector<cv::Mat> channels(3);
//    // split img:
//    cv::split(image, channels);
//    ch1 = channels[0];
//    ch2 = channels[1];
//    ch3 = channels[2];

//    test = ch1.clone();
//    test = cv::Scalar(255);
//    std::vector<cv::Mat> diff(3);
//    diff[0] = test != ch1;
//    diff[1] = test != ch2;
//    diff[2] = test != ch3;
//    for(int i = 0; i < 3; i++){
//      bool isequal = cv::countNonZero(diff[i]) == 0;
//      if(!isequal)
//        qDebug() << "\t\t ERROR #########################";
//    }
//    //
//    //end testing

    v1->setImage(Mat2QImage(image));
  }

  void ROSOutput3DWrapper::showTrackingResidual(const cv::Mat& image){
    v2->setImage(Mat2QImage(image));
  }

  void ROSOutput3DWrapper::showStereoKeyframe(const cv::Mat& image){
    v3->setImage(Mat2QImage(image));
  }

  void ROSOutput3DWrapper::showStereoReferenceFrame(const cv::Mat& image){
    v4->setImage(Mat2QImage(image));
  }

  void ROSOutput3DWrapper::publishKeyframe(Frame* f)
  {
    lsd_slam_viewer::keyframeMsg fMsg;

    boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

    fMsg.id = f->id();
    fMsg.time = f->timestamp();
    fMsg.isKeyframe = true;

    int w = f->width(publishLvl);
    int h = f->height(publishLvl);

    memcpy(fMsg.camToWorld.data(),f->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
    fMsg.fx = f->fx(publishLvl);
    fMsg.fy = f->fy(publishLvl);
    fMsg.cx = f->cx(publishLvl);
    fMsg.cy = f->cy(publishLvl);
    fMsg.width = w;
    fMsg.height = h;


    fMsg.pointcloud.resize(w*h*sizeof(InputPointDense));

    InputPointDense* pc = (InputPointDense*)fMsg.pointcloud.data();

    const float* idepth = f->idepth(publishLvl);
    const float* idepthVar = f->idepthVar(publishLvl);
    const float* color = f->image(publishLvl);

    for(int idx=0;idx < w*h; idx++)
      {
        pc[idx].idepth = idepth[idx];
        pc[idx].idepth_var = idepthVar[idx];
        pc[idx].color[0] = color[idx];
        pc[idx].color[1] = color[idx];
        pc[idx].color[2] = color[idx];
        pc[idx].color[3] = color[idx];
      }

    //boost::shared_ptr<lsd_slam_viewer::keyframeMsg const> p =
    //        boost::shared_ptr<lsd_slam_viewer::keyframeMsg const>(new lsd_slam_viewer::keyframeMsg const());

    //boost::shared_ptr<lsd_slam_viewer::keyframeMsg const> kfm_ptr(boost::make_shared<lsd_slam_viewer::keyframeMsg const>(fMsg));
    //viewer->addFrameMsg(kfm_ptr);

    viewer->addFrameMsg(&fMsg);

    //ASCOMM	keyframe_publisher.publish(fMsg);
  }

  void ROSOutput3DWrapper::publishTrackedFrame(Frame* kf)
  {
    lsd_slam_viewer::keyframeMsg fMsg;

    fMsg.id = kf->id();
    fMsg.time = kf->timestamp();
    fMsg.isKeyframe = false;


    memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
    fMsg.fx = kf->fx(publishLvl);
    fMsg.fy = kf->fy(publishLvl);
    fMsg.cx = kf->cx(publishLvl);
    fMsg.cy = kf->cy(publishLvl);
    fMsg.width = kf->width(publishLvl);
    fMsg.height = kf->height(publishLvl);

    fMsg.pointcloud.clear();

    //ASCOMM	liveframe_publisher.publish(fMsg);
    viewer->addFrameMsg(&fMsg);

    SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());

    geometry_msgs::PoseStamped pMsg;

    pMsg.pose.position.x = camToWorld.translation()[0];
    pMsg.pose.position.y = camToWorld.translation()[1];
    pMsg.pose.position.z = camToWorld.translation()[2];
    pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
    pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
    pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
    pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();

    if (pMsg.pose.orientation.w < 0)
      {
        pMsg.pose.orientation.x *= -1;
        pMsg.pose.orientation.y *= -1;
        pMsg.pose.orientation.z *= -1;
        pMsg.pose.orientation.w *= -1;
      }

    pMsg.header.stamp = ros::Time(kf->timestamp());
    pMsg.header.frame_id = "world";
    //ASCOMM	pose_publisher.publish(pMsg);*/

  }



  void ROSOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
  {
    lsd_slam_viewer::keyframeGraphMsg gMsg;

    graph->edgesListsMutex.lock();
    gMsg.numConstraints = graph->edgesAll.size();
    gMsg.constraintsData.resize(gMsg.numConstraints * sizeof(GraphConstraint));
    GraphConstraint* constraintData = (GraphConstraint*)gMsg.constraintsData.data();
    for(unsigned int i=0;i<graph->edgesAll.size();i++)
      {
        constraintData[i].from = graph->edgesAll[i]->firstFrame->id();
        constraintData[i].to = graph->edgesAll[i]->secondFrame->id();
        Sophus::Vector7d err = graph->edgesAll[i]->edge->error();
        constraintData[i].err = sqrt(err.dot(err));
      }
    graph->edgesListsMutex.unlock();

    graph->keyframesAllMutex.lock_shared();
    gMsg.numFrames = graph->keyframesAll.size();
    gMsg.frameData.resize(gMsg.numFrames * sizeof(GraphFramePose));
    GraphFramePose* framePoseData = (GraphFramePose*)gMsg.frameData.data();
    for(unsigned int i=0;i<graph->keyframesAll.size();i++)
      {
        framePoseData[i].id = graph->keyframesAll[i]->id();
        memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
      }
    graph->keyframesAllMutex.unlock_shared();

    //ASCOMM	graph_publisher.publish(gMsg);*/
    viewer->addGraphMsg(&gMsg);
  }

  void ROSOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
  {
    // unimplemented ... do i need it?
  }

  void ROSOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
  {
    // unimplemented ... do i need it?
  }

  void ROSOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
  {
    std_msgs::Float32MultiArray msg;
    for(int i=0;i<20;i++)
      msg.data.push_back((float)(data[i]));

    //ASCOMM	debugInfo_publisher.publish(msg);
  }

}

