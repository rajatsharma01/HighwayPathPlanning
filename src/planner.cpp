#include <limits>
#include "planner.h"

bool
Planner::is_current_lane_fast(const Prediction& prediction, const Car& ego) const {
    std::vector<Car> lane_cars = prediction.get_cars_in_lane(_lane, ego.get_s());
    size_t sz = lane_cars.size();

    if (sz > 0) {
        double s = lane_cars[0].get_s();
        // If car in front, check if its at adequate distance to drive faster
        if (s > ego.get_s() && (s - ego.get_s()) < FRONT_GAP) {
            // not enough gap, can't drive faster
            return false;
        }
    }

    return true;
}

bool
Planner::is_lane_change_safe(const Prediction& prediction, const Car& ego,
                             const Lane& lane, double& out_front_s) const {
    std::vector<Car> lane_cars = prediction.get_cars_in_lane(lane, ego.get_s());
    size_t sz = lane_cars.size();

    out_front_s = std::numeric_limits<double>::max();
    if (sz > 0) {
        double rear_s = 0;
        double s = lane_cars[0].get_s();

        if (s > ego.get_s()) {
            // Car in front
            out_front_s = s;
        } else {
            // Car in rear, must be a single entry vector
            rear_s = s;
            assert(sz == 1);
        }

        if (sz == 2) {
            rear_s = lane_cars[1].get_s();
            assert(rear_s < ego.get_s());
        }

        if ((out_front_s - ego.get_s()) < FRONT_GAP_LC) {
            return false;
        }
        if ((ego.get_s() - rear_s) < REAR_GAP_LC) {
            return false;
        }
    }

    return true;
}

bool
Planner::find_best_lane_change(const Prediction& prediction, const Car& ego,
                               Lane& out_lane, double& out_front_s) const {
    // Find possibility for left lane switch
    Lane left_lane;
    bool left_safe = false;
    double left_s;
    if (_lane.get_left_lane(left_lane)) {
        left_safe = is_lane_change_safe(prediction, ego, left_lane, left_s);
    }

    // Find possiblity for right lane switch
    Lane right_lane;
    bool right_safe = false;
    double right_s;
    if (_lane.get_right_lane(right_lane)) {
        right_safe = is_lane_change_safe(prediction, ego, right_lane, right_s);
    }

    // Pick safest lane change. In case of ties, pick one with farthest front car
    if (left_safe || right_safe) {
        if (left_safe && right_safe) {
            if (left_s >= right_s) {
                out_lane = left_lane;
                out_front_s = left_s;
            } else {
                out_lane = right_lane;
                out_front_s = right_s;
            }
        } else if (left_safe) {
            out_lane = left_lane;
            out_front_s = left_s;
        } else {
            out_lane = right_lane;
            out_front_s = right_s;
        }
    }

    return (left_safe || right_safe);
}

void
Planner::plan(const Prediction& prediction, const Car& ego) {
    // If its first invocation, just move this car
    if (__builtin_expect(_started, false)) {
        _speed += SPEED_INC;
        _started = false;
        return;
    }

    bool opportunistic = false;

    // If lane change is disabled, increment lane change counter,
    // it will be reset once it reaches reset count
    if (_lc_disable > 0) {
        _lc_disable = (_lc_disable + 1) % LANE_CHANGE_RESET_CNT;
       if (_lc_disable == 0) {
           std::cout << "Lane change is re-enabled" << std::endl; 
       }
    }

    // If current lane is fast, we can continue to drive in this lane
    if (is_current_lane_fast(prediction, ego)) {
        if (_speed < MAX_SPEED - SPEED_INC) {
            _speed += SPEED_INC;
        }

        // We prefer to stay in middle lane only as it is easier to switch
        // either lane from middle lane. If we are already in middle lane,
        // we are good to go.
        if (_lane.is_middle()) {
            return;
        }
        // At this point we are good to drive in current lane and no need to
        // slow down. We are opportunistically looking for lane switch to
        // middle lane, only possible lane switch at this point would lead to
        // middle lane only.
        opportunistic = true;
    }

    Lane next_lane;
    double next_s;
    if ((_lc_disable == 0) && find_best_lane_change(prediction, ego, next_lane, next_s)) {
        double s_diff = opportunistic ? FRONT_HORIZON : FRONT_GAP_LC;
        if ((ego.get_s() + s_diff) <= next_s) {
            std::cout << "Switching lane " << _lane << " --> " << next_lane << std::endl;
            std::cout << "Lane change is disabled" << std::endl;
            _lc_disable = 1; // disable lane change
            _lane = next_lane;
            return;
        }
    }

    // lane change is not possible, slow down if current lane requires us to slow down
    if (!opportunistic && _speed > SPEED_DEC) {
        _speed -= SPEED_DEC;
    }
}
