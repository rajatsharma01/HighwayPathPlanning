#ifndef _LANE_H
#define _LANE_H

#include <cassert>

class Lane {
public: // Types

    // Index of lane on highway
    enum LaneIdx {
        LANE_LEFT = 0,
        LANE_MIDDLE = 1,
        LANE_RIGHT = 2,
    };

public: // Constants
    static constexpr double LANE_WIDTH = 4.0;                   // width of the lane
    static constexpr double LANE_CENTER = LANE_WIDTH/2;         // center of lane from from lane boundary
    static const int TOTAL_LANES = 3;                           // number of lanes on highway
    static constexpr double MAX_d = LANE_WIDTH * TOTAL_LANES;   // Max value for d
    static constexpr double MAX_s = 6945.554;                   // Max value for s 

private: // Data
    LaneIdx _idx; // type of the the lane

public: // C-tor
    Lane(LaneIdx idx = LANE_MIDDLE) : _idx(idx) { }

    Lane(double d) : _idx(d_to_lane(d)) { }

private: // helpers
    LaneIdx d_to_lane(double d) const {
        assert(d >= 0 && d <= MAX_d);
        if (d < LANE_WIDTH) {
            return LANE_LEFT;
        }
        if (d < 2*LANE_WIDTH) {
            return LANE_MIDDLE;
        }
        return LANE_RIGHT;
    }

    double lane_to_d(LaneIdx idx) const {
        return static_cast<double>(idx) * LANE_WIDTH + LANE_CENTER;
    }

public: // methods
    double get_idx() const {
        return _idx;
    }

    double get_d() const {
        return lane_to_d(_idx);
    }

    bool is_left() const {
        return _idx == LANE_LEFT;
    }

    bool is_middle() const {
        return _idx == LANE_MIDDLE;
    }

    bool is_right() const {
        return _idx == LANE_RIGHT;
    }

    bool get_left_lane(Lane& out_lane) const {
        if (is_left()) {
            return false;
        }
        out_lane = Lane(static_cast<LaneIdx>(_idx-1));
        return true;
    }

    bool get_right_lane(Lane& out_lane) const {
        if (is_right()) {
            return false;
        }
        out_lane = Lane(static_cast<LaneIdx>(_idx+1));
        return true;
    }

public: // operators

    bool operator==(const Lane& other) const {
        return _idx == other._idx;
    }

    bool operator!=(const Lane& other) const {
        return !operator==(other);
    }

    bool operator<(const Lane& other) const {
        return _idx < other._idx;
    }

    bool operator>(const Lane& other) const {
        return _idx > other._idx;
    }

    friend std::ostream& operator<<(std::ostream& os, const Lane& lane) {
        os << ((lane._idx == LANE_LEFT) ? "LEFT" :
               ((lane._idx == LANE_MIDDLE) ? "MIDDLE" : "RIGHT"));
        return os;
    }
};

#endif  // _LANE_H
