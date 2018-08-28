#ifndef _PLANNER_H
#define _PLANNER_H

#include "prediction.h"

class Planner {
private: // Data
    Lane _lane;     // Driving lane updated by planner
    double _speed;  // Driving speed suggested by planner

public: // C-tor
    Planner() : _lane(), _speed(0) { }

private: // Helper methods

    // Returns true if its faster to keep driving in current lane
    bool is_current_lane_fast(const Prediction& prediction, const Car& ego) const;

    // Returns true if its safe to switch to adjascent lane
    bool is_lane_change_safe(const Prediction& prediction, const Car& ego,
                             const Lane& lane) const;

public: // methods

    // Plan a new path given predictions of other cars and predicted state
    // of ego car
    void plan(const Prediction& prediction, const Car& ego);

    // Returns d value for the target lane
    double get_d() const { return _lane.get_d(); }

    // Returns driving speed suggested by planner
    double get_speed() const { return _speed; }
};

#endif
