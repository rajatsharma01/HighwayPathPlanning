#ifndef _CAR_H
#define _CAR_H

#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include "constants.h"

// Car class can represent a logical position of a car on same side of the road
// as ego car. This class can either tell facts about current state as pulled
// from sensor fusion data or can be used as a point on trajectory predicted
// for this car.
struct Car {
private: // Data members
    double _s;          // car's s position in frenet coordinates
    double _s_dot;      // first order derivative os s w.r.t. time (longitudanal acceleration)
    double _s_dot_dot;  // second order derivative os s w.r.t. time (longitudanal jerk)
    double _d;          // car's d position in frenet coordinates
    double _d_dot;      // first order derivative of d w.r.t. time (horizontal acceleration)
    double _d_dot_dot;  // second order derivative of d w.r.t. time (horizontal jerk)

public: // C-tor
    Car() { }

    // Derived state values
    Car(double s, double s_dot, double s_dot_dot,
        double d, double d_dot, double d_dot_dot)
        : _s(s), _s_dot(s_dot), _s_dot_dot(s_dot_dot),
          _d(d), _d_dot(d_dot), _d_dot_dot(d_dot_dot)
    { }

public: // methods
    // Return new state of this car at time t
    Car at(double t) const {
        // return new position assuming constant velocity
        double t2 = t*t;
        return Car(_s + _s_dot * t + _s_dot_dot * t2 * 0.5,
                   _s_dot + _s_dot_dot * t,
                   _s_dot_dot,
                   _d + _d_dot * t + _d_dot_dot * t2 * 0.5,
                   _d_dot + _d_dot_dot * t,
                   _d_dot_dot);
    }

    // Accessors
    double get_s() const { return _s; }
    double get_d() const { return _d; }
    double get_s_dot() const { return _s_dot; }
    double get_d_dot() const { return _d_dot; }
    double get_s_dot_dot() const { return _s_dot_dot; }
    double get_d_dot_dot() const { return _d_dot_dot; }

    std::vector<double> get_s_vector() const {
        std::vector<double> svec = {_s, _s_dot, _s_dot_dot};
        return svec;
    }

    std::vector<double> get_d_vector() const {
        std::vector<double> dvec = {_d, _d_dot, _d_dot_dot};
        return dvec;
    }

public: // operators
    bool operator==(const Car& other) const {
        return (_s == other._s && _s_dot == other._s_dot && _s_dot_dot == other._s_dot_dot &&
                _d == other._d && _d_dot == other._d_dot && _d_dot_dot == other._d_dot_dot);
    }

    bool operator!=(const Car& other) const {
        return !operator==(other);
    }

    bool operator<(const Car& other) const {
        return (_s < other._s);
    }

    bool operator>(const Car& other) const {
        return (_s > other._s);
    }

private: // friends
    friend std::ostream& operator<<(std::ostream& os, const Car& car) {
        std::vector<std::vector<double>> sd_vec = { car.get_s_vector(), car.get_d_vector() };
        os <<"Car: ";
        for (size_t i = 0; i < sd_vec.size(); i++) {
            os << ((i == 0) ? "s" : ", d") << " vector: [ ";
            for (size_t ii = 0; ii < sd_vec[i].size(); ii++) {
                if (ii > 0) {
                    os << ", ";
                }
                os << sd_vec[i][ii];
            }
            os << " ]";
        }
        return os;
    }
};

// Predictions of all cars state on the road
typedef std::map<int, Car> Predictions;

#endif // _CAR_H
