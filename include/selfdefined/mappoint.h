#ifndef MAPPOINT_H
#define MAPPOINT_H
#include "common_include.h"
#include "feature.h"

namespace selfdefined {

struct Frame;
struct Feature;

struct MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    // 路标点属性
    unsigned long id = 0; 
    bool is_outlier = false;
    Vec3 pos = Vec3::Zero();
    std::mutex data_mutex;
    int observed_times = 0;
    std::list<std::weak_ptr<Feature>> observations; // 观测到该路标点的所有feature
    
    // constructor
    MapPoint() {}

    MapPoint(long id, Vec3 pos) {
        this->id = id;
        this->pos = pos;
    }

    // getter and setter functions
    // 数据锁防止pose数据混乱
    void setPos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lck(data_mutex);
        this->pos = pos;
    }

    Vec3 getPos() {
        std::unique_lock<std::mutex> lck(data_mutex);
        return pos;
    }

    // 添加新的feature
    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex);
        observations.push_back(feature);
        observed_times++;
    }

    // delete the feature in the observation
    void RemoveObservation(std::shared_ptr<Feature> feat) {
        std::unique_lock<std::mutex> lck(data_mutex);
        for (auto iter = observations.begin(); iter != observations.end(); iter++) {
            if (iter->lock() == feat) {
                observations.erase(iter);
                feat->map_point.reset();
                observed_times--;
                break;
            }
        }
    }

    // return observation
    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex);
        return observations;
    }

    static MapPoint::Ptr CreateNewMappoint() {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id = factory_id++;
        return new_mappoint;
    }



};

}



#endif