#include "map.h"
#include "feature.h"
#include "frame.h"

namespace selfdefined{

// insert it in the map
void Map::insertKeyFrame(Frame::Ptr frame) {
    this->current_frame = frame;
    // if the frame is not in keyframe, then insert it in keyframe
    if (keyframes.find(frame->keyframe_id) == keyframes.end()) {
        keyframes.insert(make_pair(frame->keyframe_id, frame));
        active_keyframes.insert(make_pair(frame->keyframe_id, frame));
    }
    // if find, replace it
    else {
        keyframes[frame->keyframe_id] = frame;
        active_keyframes[frame->keyframe_id] = frame;
    }

    if (active_keyframes.size() > num_of_active_frames) {
        removeOldKeyFrame();
    }

}
// insert it in the map
void Map::insertMapPoint(MapPoint::Ptr mapPoint) {
    if (landmarks.find(mapPoint->id) == landmarks.end()) {
        landmarks.insert(make_pair(mapPoint->id, mapPoint));
        active_landmarks.insert(make_pair(mapPoint->id, mapPoint));
    } else {
        landmarks[mapPoint->id] = mapPoint;
        active_landmarks[mapPoint->id] = mapPoint;
    }
}

// clean the landmarks that are not observed anymore
void Map::cleanMap() {
    int cnt_landmark_removed = 0;
    for (auto it = active_landmarks.begin(); it != active_landmarks.end();) {
        if (it->second->observed_times == 0) {
            it = active_landmarks.erase(it);
            cnt_landmark_removed++;
        }
        else {
            it++;
        }
    }
}

void Map::removeOldKeyFrame() {
    if (current_frame == nullptr) return;

    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame->getPose().inverse();
    // select the farthest and nearest frame id
    for (auto& kf: active_keyframes) {
        if (kf.second == current_frame) continue;
        auto dis = (kf.second->getPose() * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    // chose to remove farthest or nearest frame
    const double min_dis_th = 0.2;
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        frame_to_remove = keyframes.at(min_kf_id);
    } else {
        frame_to_remove = keyframes.at(max_kf_id);
    }

    // erase the frame and features in that frame
    active_keyframes.erase(frame_to_remove->id);
    for (auto feat : frame_to_remove->features_left) {
        auto mp = feat->map_point.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }

    for (auto feat : frame_to_remove->features_right) {
        if (feat == nullptr) continue;
        auto mp = feat->map_point.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }

    cleanMap();
}

}