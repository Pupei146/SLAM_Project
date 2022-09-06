#ifndef FRONTEND_H
#define FRONTEND_H

#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace selfdefined {
class Backend;
class Viewer;

enum class FrontendStatus {INITING, TRACKING_GOOD, TRACKING_BAD, LOST};

class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;
    
    Frontend(); //constructor

    bool addFrame(Frame::Ptr frame);

    void setMap(Map::Ptr map) { map_ = map; }

    void setBackend(std::shared_ptr<Backend> backend) { backend = backend; }

    void setViewer(std::shared_ptr<Viewer> viewer) { viewer = viewer; }

    FrontendStatus GetStatus() const { return status; }

    void setCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left = left;
        camera_right = right;
    }


private:
    bool stereoInit();

    bool track();

    int trackLastFrame();

    int estimateCurrentPose();

    bool reset();

    bool insertKeyframe();

    int detectFeatures();

    int findFeaturesInRight();

    bool buildInitMap();

    int triangulateNewPoints();

    void setObservationsForKeyFrame();

    FrontendStatus status = FrontendStatus::INITING;

    Frame::Ptr current_frame = nullptr;  // 当前帧
    Frame::Ptr last_frame = nullptr;     // 上一帧
    Camera::Ptr camera_left = nullptr;   // 左侧相机
    Camera::Ptr camera_right = nullptr;  // 右侧相机

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend = nullptr;
    std::shared_ptr<Viewer> viewer = nullptr;

    SE3 relative_motion;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers = 0;  // inliers, used for testing new keyframes

    // params
    int num_features = 200;
    int num_features_init = 100;
    int num_features_tracking = 50;
    int num_features_tracking_bad = 20;
    int num_features_needed_for_keyframe = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt;  // feature detector in opencv

};


}



#endif