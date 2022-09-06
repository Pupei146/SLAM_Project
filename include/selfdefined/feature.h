#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "common_include.h"

namespace selfdefined {
// declearation
struct Frame;
struct MapPoint;

struct Feature {
public:
    // constructors
    Feature() {}
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &p) {
        
    }

public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame; // freams that contain this feature
    std::weak_ptr<MapPoint> map_point; // 关联路标点

    cv::KeyPoint position; // 2D position of the feature;
    

    bool is_outlier = false;
    bool is_on_left = true;

};
}

#endif