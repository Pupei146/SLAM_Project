#include "camera.h"
namespace selfdefined
{
Camera::Camera() {}

// coordinate transform between world, camera, and pixel
// 3D point in world transfered to camera frame
Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w) {
    return pose * T_c_w * p_w;
}

// 3D point in world transfered to camera frame
Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w) {
    return T_c_w.inverse() * pose_inv * p_c;
}

// 3D point in camera frame to image frame
Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    int u = p_c(0,0) / p_c(0,2) * fx + cx;
    int v = p_c(0,1) / p_c(0,2) * fy + cy;
    Vec2 ans(u, v);
    return ans;
}

// 2D point in image coordinate to 3D point in camera frame
Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth) {
    int x = (p_p(0,0) - cx) / fx * depth;
    int y = (p_p(0,0) - cx) / fx * depth;
    Vec3 ans(x, y, depth);
    return ans;
}

// 2D point in image coordinate to 3D point in world frame
Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));

}
} // namespace selfdefined
