#ifndef _PREDICTION_H
#define _PREDICTION_H

#include <map>
#include "car.h"
#include "lane.h"

class Prediction {
private: // Types
    // A map of cars ordered by their s values as key
    typedef std::map<double, Car> CarMap;

    // A vector cotaining map for cars for each lane
    typedef std::vector<CarMap> LaneCarMap;

private: // Data
    // A map of cars for each lane at time of creation
    LaneCarMap _lane_cars;

public: // C-tor
    Prediction() : _lane_cars(Lane::TOTAL_LANES) { }

public: // Methods

    // Add a new car to its lane based on car's d value
    // return false if car's s or d value are out of range
    bool add(const Car& car);

    // Get the cars at distance 's' in 'lane'.
    // Inputs:
    //  lane    -   lookup lane
    //  s       -   distance along the lane where we are looking for cars
    // Returns a vector of lenght atmost 2, with following number of cars
    //  0   -   no cars were found in this lane
    //  1   -   Only car behind (< s) or ahead (> s)
    //  2   -   Two cars, first is behind (< s), second is ahead (> s)
    std::vector<Car> get_cars_in_lane(const Lane& lane, double s) const;
};

#endif  // _PREDICTION_H
