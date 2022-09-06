#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"

namespace selfdefined {

class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // constructor
    Camera();
    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose) {
        this->fx = fx;
        this->fy = fy;
        this->cx = cx;
        this->cy = cy;
        this->baseline_ - baseline;
        this->pose = pose;
        this->pose_inv = pose.inverse();
    }

    ~Camera(); // destructor

    // Helper Function
    // get intrisic parameter
    Mat33 get_intrisic() {
        Mat33 K;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        return K;
    }

    // get extrinsic
    SE3 get_extrinsic() {
        return pose;
    }

    SE3 get_extrinsic_inv() {
        return pose_inv;
    }

    typedef std::shared_ptr<Camera> Ptr;

    // coordinate transform between world, camera, and image frame
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);

private:
    // Camera intrinsics
    double fx = 0;
    double fy = 0; 
    double cx = 0; 
    double cy = 0;
    double baseline_ = 0;  

    SE3 pose;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv;         // inverse of extrinsics




};

}

#endif