#ifndef MAP_H
#define MAP_H
#include "frame.h"
#include "feature.h"
#include "mappoint.h"

namespace selfdefined {
class Map {
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> landMarkMap;
    typedef std::unordered_map<unsigned long, Frame::Ptr> keyFrameMap;

    Map() {}

    // getter and setter functions
    landMarkMap getAllLandmarks() {
        std::unique_lock<std::mutex> lck(data_mutex);
        return landmarks;
    }

    landMarkMap getAllActiveLandmarks() {
        std::unique_lock<std::mutex> lck(data_mutex);
        return active_landmarks;
    }

    keyFrameMap getAllKeyframes() {
        std::unique_lock<std::mutex> lck(data_mutex);
        return keyframes;
    }

    keyFrameMap getAllActiveKeyframes() {
        std::unique_lock<std::mutex> lck(data_mutex);
        return active_keyframes;
    }

    void insertKeyFrame(Frame::Ptr keyFrame);

    void insertMapPoint(MapPoint::Ptr mapPoint);

    void cleanMap();

private:
    void removeOldKeyFrame();

    std::mutex data_mutex;
    
    landMarkMap landmarks;
    landMarkMap active_landmarks;
    keyFrameMap keyframes;
    keyFrameMap active_keyframes;

    Frame::Ptr current_frame = nullptr;

    int num_of_active_frames = 7;

};



} // namespace selfdefined





#endif