#include "trajectory_split.h"
#include <cmath>
#include <vector>

void split_traj(Trajectory* traj, int tedge_id) {
    double x1, x2, y1, y2;
    Tedge* tedge = traj -> edges[tedge_id];
    x1 = tedge -> src -> latitude; 
    y1 = tedge -> src -> longitude;
    x2 = tedge -> trg -> latitude;
    y2 = tedge -> trg -> longitude;
    double length_new = tedge -> cost / 2; 

    Point* pt = (Point*) malloc(sizeof(Point));
    Tedge* tedge2 = (Tedge*) malloc(sizeof(Tedge));

    pt -> latitude= (x1+x2)/2;   
    pt -> longitude = (y1+y2)/2;
    traj -> points.push_back(pt);

    tedge2 -> src = pt; 
    tedge2 -> trg = tedge -> trg;
    tedge -> trg = pt; //the original target node
    tedge -> cost = length_new ;
    tedge2 -> cost = length_new ;
    auto it = traj -> edges.insert(traj -> edges.begin() + tedge_id + 1, tedge2);

return;
}


void subsample_traj(Trajectory* traj, double threshold){
    for (int i = 0; i <  traj -> edges.size(); i++) {  
        if (traj -> edges[i] -> cost > threshold) {
            while (traj -> edges[i] -> cost > threshold) {
                split_traj(traj, i);        
            } 
        }
    }
    return;
}







































