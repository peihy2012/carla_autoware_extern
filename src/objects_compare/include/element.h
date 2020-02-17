#pragma once

#include <iostream>
#include <string>
#include <queue>
#include <list>

namespace ele {

struct Vector2D {
    double x;
    double y;
};

struct Vector3D {
    double x;
    double y;
    double z;
};

struct Vector4D {
    double x;
    double y;
    double z;
    double w;
};

// position in 2D
// struct Pose2D {
//     double x;
//     double y;
// };    
typedef struct Vector2D Pose2D;

typedef struct Vector3D Pose3D;

typedef struct Vector2D Dimension2D;

typedef struct Vector3D Dimension3D;

typedef struct Vector2D Velocity2D;

typedef struct Vector3D Velocity3D;

typedef struct Vector2D Accel2D;

typedef struct Vector3D Accel3D;

// quaternion in XY-plain with x=0 y=0
struct Orientation2D {
    // qz = az * sin(angle/2)
    double z;
    // qw = cos(angle/2)
    double w;
};

typedef struct Vector4D Orientation3D;

struct Item {
    double time;
    double angle;    
    Pose2D pose;
    Dimension2D dimension;
    Velocity2D velocity;
    Accel2D accel;
};

template <typename T, size_t ItemMaxSize>
class Trajectory {
public:
    Trajectory(): updated_(false), life_time_(0), lost_time_(0) {}
    Trajectory(const unsigned int id_in, T& item_in)
        : id_(id_in), updated_(true), life_time_(1), lost_time_(0) {
        items_.push_back(item_in);
    }
    // ~Trajectory() {}
    void pop () {
        if (updated_) {
            updated_ = false;
            if (items_.size() > ItemMaxSize) {
                items_.pop_front();
            }
        } else {
            ++lost_time_;
            if ((items_.size() + lost_time_) > ItemMaxSize) {
                if (!items_.empty()) {
                    items_.pop_front();                    
                }
            }
        }
    }
    void push (T& item_in) {
        life_time_++;
        items_.push_back(item_in);
        updated_ = true;
    }
    unsigned int id () const {
        return id_;
    }
    bool empty () {
        return items_.empty();
    }

    unsigned int id_;    
    bool updated_;
    unsigned int life_time_;
    unsigned int lost_time_;
    std::list<T> items_;
};

template <typename T, size_t ItemMaxSize>
class TrajectoryList {
public:
    TrajectoryList(): size_(0) {}

    void push (const unsigned int id, T& item) {
        for (auto it = trajectories_.begin(); it != trajectories_.end(); ++it) {
            if (it->id() == id) {
                it->push(item);
                return ;
            }
        }
        // Trajectory<T, ItemMaxSize> new_trajectory(id, item);
        // trajectories_.push_back( new_trajectory );
        trajectories_.push_back( Trajectory<T, ItemMaxSize> (id, item) );
        ++size_;
    }

    void pop () {
        for (auto it = trajectories_.begin(); it != trajectories_.end(); ) {
            it->pop();
            if (it->empty()) {
                it = trajectories_.erase(it);
                --size_;
            }
        }
    }
 
    unsigned int size_;
    std::list<Trajectory<T, ItemMaxSize> > trajectories_;
};


} // namespace ele
