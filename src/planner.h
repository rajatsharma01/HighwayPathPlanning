#ifndef _PLANNER_H
#define _PLANNER_H

#include "prediction.h"

class Planner {
private: // Data
    Lane _lane;         // Driving lane updated by planner
    double _speed;      // Driving speed suggested by planner
    bool _started;      // First invocation of plan
    int _lc_disable;    // If > 0, lane change is disabled. Reset once it reaches
                        // LANE_CHANGE_RESET_CNT

public: // C-tor
    Planner() : _lane(), _speed(0), _started(true), _lc_disable(0) { }

private: // Helper methods

    // Returns true if its faster to keep driving in current lane.
    bool is_current_lane_fast(const Prediction& prediction, const Car& ego) const;

    // Returns true if its safe to switch to adjascent lane, out_front_s is
    // updated with s distance of front car.
    bool is_lane_change_safe(const Prediction& prediction, const Car& ego,
                             const Lane& lane, double& out_front_s) const;

    // Finds best lane change possible based on distance to front car and updates
    // out_lane and out_front_s with chosen lane and position of front car in
    // chosen lane. Returns false if lane change is not feasible.
    bool find_best_lane_change(const Prediction& prediction, const Car& ego,
                               Lane& out_lane, double& out_front_s) const;
public: // methods

    // Plan a new path given predictions of other cars and predicted state
    // of ego car.
    void plan(const Prediction& prediction, const Car& ego);

    // Returns d value for the target lane.
    double get_d() const { return _lane.get_d(); }

    // Returns driving speed suggested by planner.
    double get_speed() const { return _speed; }
};

#endif
