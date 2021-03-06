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

#pragma once

//#include <ros/ros.h>
#include "IOWrapper/Output3DWrapper.h"

#include "lsd_slam_viewer/QGLDisplay.h"

namespace lsd_slam
{


  class Frame;
  class KeyFrameGraph;

  struct InputPointDense
  {
    float idepth;
    float idepth_var;
    unsigned char color[4];
  };

  struct GraphConstraint
  {
    int from;
    int to;
    float err;
  };

  struct GraphFramePose
  {
    int id;
    float camToWorld[7];
  };



  /// Addition to LiveSLAMWrapper for ROS interoperability.
  class ROSOutput3DWrapper : public Output3DWrapper
  {
  public:

    // initializes cam-calib independent stuff
    ROSOutput3DWrapper(int width, int height);
    ~ROSOutput3DWrapper();

    virtual void setViewer(PointCloudViewer *v){this->viewer = v;}

    virtual void setViews(QGLDisplay *v1, QGLDisplay *v2, QGLDisplay *v3, QGLDisplay *v4)
    {
      this->v1=v1; this->v2=v2;this->v3=v3; this->v4=v4;
      v1->setWindowTitle("Debug Depth");
      v2->setWindowTitle("Tracking Residual");
      v3->setWindowTitle("Stereo Keyframe");
      v4->setWindowTitle("Stereo Reference Frame");
    };

    virtual void showKeyframeDepth( const cv::Mat& image);
    virtual void showTrackingResidual( const cv::Mat& image);
    virtual void showStereoKeyframe( const cv::Mat& image);
    virtual void showStereoReferenceFrame( const cv::Mat& image);

    virtual void publishKeyframeGraph(KeyFrameGraph* graph);

    // publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
    virtual void publishKeyframe(Frame* f);

    // published a tracked frame that did not become a keyframe (i.e. has no depth data)
    virtual void publishTrackedFrame(Frame* f);

    // publishes graph and all constraints, as well as updated KF poses.
    virtual void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier);

    virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier);

    virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data);

    int publishLvl;

  private:

    // Viewer for 3D reconstructed scene, uses qglviewer
    PointCloudViewer* viewer;

    // Views for image streams, altogether 4 image stream will be displayed
    QGLDisplay *v1;
    QGLDisplay *v2;
    QGLDisplay *v3;
    QGLDisplay *v4;

    int width, height;

    std::string liveframe_channel;
    //ros::Publisher liveframe_publisher;

    std::string keyframe_channel;
    //ros::Publisher keyframe_publisher;

    std::string graph_channel;
    //ros::Publisher graph_publisher;

    std::string debugInfo_channel;
    //ros::Publisher debugInfo_publisher;

    std::string pose_channel;
    //ros::Publisher pose_publisher;

    //ros::NodeHandle nh_;
  };
}
