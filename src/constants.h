#ifndef _CONSTANTS_H
#define _CONSTANTS_H

static const double NUM_PATH_POINTS = 30;   // Number of path points
static const double TIME_STEP = 0.02;       // Simulator executes point in trajectory at each time step in seconds

static const double MAX_SPEED = 22.22;      // maximum allowed speed in m/s (= 49.7 MPH)
static const double SPEED_INC = 0.2;        // Increment speed by this step value
static const double SPEED_DEC = 0.2;        // Decmrenet speed by this step value

static const double FRONT_GAP = 30;         // preferred gap in m to maintain from front car
static const double REAR_GAP_LC = 30;       // minimum gap in m from rear car for lane change
static const double FRONT_GAP_LC =
                    FRONT_GAP + 10;         // minimum gap in m from front car for lane change

static const double FRONT_HORIZON = 500;    // How far in front do we look for other cars
static const double REAR_HORIZON = 100;     // How far in rear do we look for other cars

static const int NUM_WAY_POINTS = 3;        // Number of way points to create trajectory
static const double SPACE_IN_WAYPOINTS =
                    FRONT_GAP_LC + 10;       // Space between waypoints

static const int LANE_CHANGE_RESET_CNT = 50; // Counter to avoid too frequent lane changes

#endif  // _CONSTANTS_H
