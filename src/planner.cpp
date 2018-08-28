#include <limits>
#include "planner.h"

bool
Planner::is_current_lane_fast(const Prediction& prediction, const Car& ego) const {
    std::vector<Car> lane_cars = prediction.get_cars_in_lane(_lane, ego.get_s());
    size_t sz = lane_cars.size();

    if (sz > 0) {
        double s = lane_cars[0].get_s();
        if (s < ego.get_s() || (s - ego.get_s()) > FRONT_GAP) {
            // Car in front is at a distance, we can drive faster
            return true;
        }
    }

    return false;
}

bool
Planner::is_lane_change_safe(const Prediction& prediction, const Car& ego,
                             const Lane& lane) const {
    std::vector<Car> lane_cars = prediction.get_cars_in_lane(lane, ego.get_s());
    size_t sz = lane_cars.size();

    if (sz > 0) {
        double front_s = std::numeric_limits<double>::max();
        double rear_s = 0;

        double s = lane_cars[0].get_s();
        if (s > ego.get_s()) {
            // Car in front
            front_s = s;
        } else {
            // Car in rear, must be a single entry vector
            rear_s = s;
            assert(sz == 1);
        }

        if (sz == 2) {
            rear_s = lane_cars[1].get_s();
            assert(rear_s < ego.get_s());
        }

        if (front_s - ego.get_s() < FRONT_GAP) {
            return false;
        }
        if ((ego.get_s() - rear_s) < REAR_GAP) {
            return false;
        }
    }


    return true;
}

void
Planner::plan(const Prediction& prediction, const Car& ego) {
    bool slow_down = true;

    // If current lane is fast, we can continue to drive in this lane
    if (is_current_lane_fast(prediction, ego)) {
        if (_speed < MAX_SPEED - SPEED_INC) {
            _speed += SPEED_INC;
        }
        // Prefer to stay in middle lane only
        if (_lane.is_middle()) {
            return;
        }
        // It's not a middle lane, switch over to middle lane if possible.
        // Don't slow down if lane switch is not possible, we can still
        // drive faster in current lane.
        slow_down = false;
        return; // FIXME Currently this results into frequently switching back and forth to middle lane
    }

    // Current lane is not safe to drive, try to switch over to left lane
    Lane left_lane;
    if (_lane.get_left_lane(left_lane)) {
        if (is_lane_change_safe(prediction, ego, left_lane)) {
            std::cout << "Moving left from: " << _lane << " to: " << left_lane << std::endl;
            _lane = left_lane;
            return;
        }
    }

    // If left lane in not safe either, try to switch over to right lane
    Lane right_lane;
    if (_lane.get_right_lane(right_lane)) {
        if (is_lane_change_safe(prediction, ego, right_lane)) {
            std::cout << "Moving right from: " << _lane << " to: " << right_lane << std::endl;
            _lane = right_lane;
            return;
        }
    }

    // lane change is not possible, slow down if current lane requires us to slow down
    if (slow_down && _speed > SPEED_DEC) {
        _speed -= SPEED_DEC;
    }
}
