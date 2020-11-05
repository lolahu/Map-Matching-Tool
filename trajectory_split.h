#ifndef TRAJECTORY_SPLIT_H
#define TRAJECTORY_SPLIT_H

#include <iostream> 
#include <fstream> 
#include <cmath>
#include <cstdlib>
#include <vector>
#include "scale_projection.h"
#include "trajectory.h"

#define TRAJ_VAL_SIZE (4)
#define LON_LAT_COMMA_SHIFT (7)
#define DEF_TRAJ {0, 0, 0};

using namespace std;

/* adds a trajectory edge */
void split_traj(Trajectory* traj, int tedge_id);

/* subsample a trajectory */
void subsample_traj(Trajectory* traj, double threshold);

#endif