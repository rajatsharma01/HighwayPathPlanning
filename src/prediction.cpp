#include "prediction.h"

bool
Prediction::add(const Car& car) {
    double d = car.get_d();
    if (d < 0 || d > Lane::MAX_d) {
        return false;
    }
    double s = car.get_s();
    if (s < 0 || s > Lane::MAX_s) {
        return false;
    }

    _lane_cars[Lane(d).get_idx()].insert(std::make_pair(s, car));
    return true;
}

std::vector<Car>
Prediction::get_cars_in_lane(const Lane& lane, double s) const {
    std::vector<Car> car_vec;

    // Get cars in lane
    const CarMap& cmap = _lane_cars[lane.get_idx()];
    CarMap::const_iterator ahead = cmap.lower_bound(s);
    if (ahead != cmap.end()) {
        car_vec.push_back(ahead->second);
        if (ahead != cmap.begin()) {
            // Found a car behind
            CarMap::const_iterator prev = std::prev(ahead);
            car_vec.push_back(prev->second);
        }
    }

    return car_vec;
}
