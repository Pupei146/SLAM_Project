#include <opencv2/opencv.hpp>

#include "algorithm.h"
#include "backend.h"
#include "config.h"
#include "feature.h"
#include "frontend.h"
#include "g2o_types.h"
#include "map.h"
#include "viewer.h"

namespace selfdefined{
// constructor 
Frontend::Frontend() {
    // get the feature parameter from the config file
    gftt = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    
    num_features_init = Config::Get<int>("num_features_init");
    num_features = Config::Get<int>("num_features");
}

bool Frontend::track() {
    // 匀速直线运动，将上一个位置与上一个位置与上上个位置的关系计算当前位置
    if (last_frame) {
        current_frame->setPose(relative_motion * last_frame->getPose());
    }
    int num_track_last = trackLastFrame(); // 匹配上一帧与当前帧，返回匹配到的点的数量


}

int Frontend::trackLastFrame() {
    std::vector<cv::Point2f> kps_last, kps_current; // 上一帧的keypoints与上一帧的keypoints对应的map point到平面的重投影
    for (auto &kp : last_frame->features_left) {
        kps_last.push_back(kp->position.pt);
        // 如果该keypoints有对应的map point，计算重投影
        if (kp->map_point.lock()) {
            auto mp = kp->map_point.lock();
            auto px = camera_left->world2pixel(mp->getPos(), current_frame->getPose());
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        }
        // 如果没有map point，直接使用key point的坐标
        else {                                            
            kps_current.push_back(kp->position.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;

    // 使用opencv自带的LK光流法
    cv::calcOpticalFlowPyrLK(last_frame->left_image, current_frame->left_image, kps_last, kps_current, status, error, 
        cv::Size(11, 11), 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
    // calcOpticalFlowPyrLK(匹配图1，匹配图2，图1关键点，图2关键点存放容器（可以有初值）
    // 匹配情况，匹配误差，金字塔窗口大小，金字塔层数，终止条件（迭代次数+最小步长），
    // 使用图2关键点存放容器中的初值)

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) { // 匹配成功
            cv::KeyPoint kp(kps_current[i], 7);// 关键点代表的区域直径大小为7
            // 有一个关键点，就要有一个特征类
            Feature::Ptr feature(new Feature(current_frame, kp));
            // 关键点对应的地图点就是上一帧的点对应的地图点
            feature->map_point = last_frame->features_left[i]->map_point;  
            // 填充当前帧的左图特征点容器 
            current_frame->features_left.push_back(feature);      
            num_good_pts++;             // 匹配成功点计数
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;

}

int Frontend::estimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame->getPose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left->get_intrisic();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame->features_left.size(); ++i) {
        auto mp = current_frame->features_left[i]->map_point.lock();
        if (mp) {
            features.push_back(current_frame->features_left[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame->features_left[i]->position.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the current pose and determine outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame->getPose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);    // 每次循环迭代10次
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier) { // 特征点本身就是异常点，计算重投影误差
                e->computeError();
            }
// （信息矩阵对应的范数）误差超过阈值，判定为异常点，并计数，否则恢复为正常点
            if (e->chi2() > chi2_th) {      
                features[i]->is_outlier = true;
// 设置等级  一般情况下g2o只处理level = 0的边，设置等级为1，下次循环g2o不再优化异常值
                e->setLevel(1);                     
                cnt_outlier++;
            } else {
                features[i]->is_outlier = false;
                e->setLevel(0);
            };  // 这里为什么加分号意义不明，测试不加也是能正常运行的。有知道的欢迎评论

            if (iteration == 2) {
// 最后一次循环时去掉鲁棒核函数
                e->setRobustKernel(nullptr);        
            }
        }
    }
    // 异常点/正常点记录到日志
    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // 修正当前帧的位姿
    current_frame->setPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame->getPose().matrix();
// 对于被认为是异常值的特征，重置特征与路标点的对应关系（而不是重置路标点）
// 并把它重新记作正常值，认为它只是对应关系错了，并不是所谓的噪点，可能未来有用
    for (auto &feat : features) {
        if (feat->is_outlier) {
            feat->map_point.reset();
            feat->is_outlier = false;  // maybe we can still use it in future
        }
    }
// 返回正常值数量，也就是认为正确追踪到的点的数量
    return features.size() - cnt_outlier;
}


bool Frontend::stereoInit() {
    int num_features_left = detectFeatures();
    int num_coor_features = findFeaturesInRight();
    if (num_coor_features < num_features_init) {
        return false;
    }

    bool build_map_success = buildInitMap();
    if (build_map_success) {
        status = FrontendStatus::TRACKING_GOOD;
        if (viewer) {
            viewer->AddCurrentFrame(current_frame);
            viewer->UpdateMap();
        }
        return true;
    }
    return false;

}


// add a new frame
bool Frontend::addFrame(Frame::Ptr frame) {
    current_frame = frame;
    switch(status) {
        case FrontendStatus::INITING:

    }
}

bool Frontend::reset() {
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}

bool Frontend::insertKeyframe() {
    if (tracking_inliers >= num_features_needed_for_keyframe) {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame->setKeyFrame();
    map_->insertKeyFrame(current_frame);

    LOG(INFO) << "Set frame " << current_frame->id << " as keyframe "
              << current_frame->keyframe_id;

    setObservationsForKeyFrame();
    detectFeatures();  // detect new features

    // track in right image
    findFeaturesInRight();
    // triangulate map points
    triangulateNewPoints();
    // update backend because we have a new keyframe
    this->backend->UpdateMap();

    if (viewer) viewer->UpdateMap();

    return true;
}

int Frontend::findFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame->features_left) {
        kps_left.push_back(kp->position.pt);
        auto mp = kp->map_point.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                camera_right->world2pixel(mp->pos, current_frame->getPose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame->left_image, current_frame->right_image, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame, kp));
            feat->is_on_left = false;
            current_frame->features_right.push_back(feat);
            num_good_pts++;
        } else {
            current_frame->features_right.push_back(nullptr);
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

bool Frontend::buildInitMap() {
    std::vector<SE3> poses{camera_left->get_extrinsic(), camera_right->get_extrinsic()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame->features_left.size(); ++i) {
        if (current_frame->features_right[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left->pixel2camera(
                Vec2(current_frame->features_left[i]->position.pt.x,
                     current_frame->features_left[i]->position.pt.y)),
            camera_right->pixel2camera(
                Vec2(current_frame->features_right[i]->position.pt.x,
                     current_frame->features_right[i]->position.pt.y))};
        Vec3 pworld = Vec3::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->setPos(pworld);
            new_map_point->AddObservation(current_frame->features_left[i]);
            new_map_point->AddObservation(current_frame->features_right[i]);
            current_frame->features_left[i]->map_point = new_map_point;
            current_frame->features_right[i]->map_point = new_map_point;
            cnt_init_landmarks++;
            map_->insertMapPoint(new_map_point);
        }
    }
    current_frame->setKeyFrame();
    map_->insertKeyFrame(current_frame);
    backend->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

int Frontend::triangulateNewPoints() {
    std::vector<SE3> poses{camera_left->get_extrinsic(), camera_right->get_extrinsic()};
    SE3 current_pose_Twc = current_frame->getPose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame->features_left.size(); ++i) {
        if (current_frame->features_left[i]->map_point.expired() &&
            current_frame->features_right[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left->pixel2camera(
                    Vec2(current_frame->features_left[i]->position.pt.x,
                         current_frame->features_left[i]->position.pt.y)),
                camera_right->pixel2camera(
                    Vec2(current_frame->features_right[i]->position.pt.x,
                         current_frame->features_right[i]->position.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->setPos(pworld);
                new_map_point->AddObservation(
                    current_frame->features_left[i]);
                new_map_point->AddObservation(
                    current_frame->features_right[i]);

                current_frame->features_left[i]->map_point = new_map_point;
                current_frame->features_right[i]->map_point = new_map_point;
                map_->insertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

void Frontend::setObservationsForKeyFrame() {
    for (auto &feat : current_frame->features_left) {
        auto mp = feat->map_point.lock();
        if (mp) mp->AddObservation(feat);
    }
}

}