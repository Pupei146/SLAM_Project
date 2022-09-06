#ifndef FRAME_H
#define FRAME_H
#include "camera.h"
#include "common_include.h"

namespace selfdefined
{
// declaration
struct MapPoint;
struct Feature;

struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // 
    typedef std::shared_ptr<Frame> Ptr;

    // constructor
    Frame() {}
    Frame(unsigned long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right) {
        this->id = id;
        this->time_stamp = time_stamp;
        this->pose = pose;
        this->left_image = left;
        this->right_image = right; 
    }

    // Pose getter and setter functions
    // 上锁为了防止前后段访问出现竞争问题
    SE3 getPose() {
        std::unique_lock<std::mutex> lck(pose_mutex);
        return this->pose;
    }


    void setPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex);
        this->pose = pose;
    }

    void setKeyFrame() {
        static long keyframe_factory_id = 0;
        this->is_keyframe = true;
        keyframe_id = keyframe_factory_id++;
    }

    // 工厂构建模式，分配ID
    static Frame::Ptr createFrame() {
        static long factory_id = 0;
        Frame::Ptr new_frame(new Frame);
        new_frame->id = factory_id++;
        return new_frame;
    }

    unsigned long id = 0;
    unsigned long keyframe_id = 0;
    
    
    double time_stamp; // not in use
    
    cv::Mat left_image, right_image; 

    // features in left image
    std::vector<std::shared_ptr<Feature>> features_left;
    // features in right image
    std::vector<std::shared_ptr<Feature>> features_right;


    // destructor
    ~Frame() {}

public:
    SE3 pose; // Trans from world to camera (left)
    std::mutex pose_mutex; // pose 数据锁
    bool is_keyframe = false; 
    
};
} // namespace slefdefined


#endif