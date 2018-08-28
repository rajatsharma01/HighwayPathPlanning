#ifndef _CONSTANTS_H
#define _CONSTANTS_H

static const double NUM_PATH_POINTS = 50;   // Number of path points
static const double TIME_STEP = 0.02;       // Simulator executes point in trajectory at each time step in seconds

static const int NUM_WAY_POINTS = 3;        // Number of way points to create trajectory
static const double SPACE_IN_WAYPOINTS = 30; // Space between waypoints

static const double MAX_SPEED = 22.22;      // maximum allowed speed in m/s (= 49.7 MPH)
static const double SPEED_INC = 0.2;        // Increment speed by this step value
static const double SPEED_DEC = 0.2;        // Decmrenet speed by this step value

static const double FRONT_GAP = 25;         // gap in m to maintain from front car
static const double REAR_GAP = 15;          // gap in m to maintain from rear car

#endif  // _CONSTANTS_H
