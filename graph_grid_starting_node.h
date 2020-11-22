#ifndef GRAPH_GRID_STARTING_NODE_H
#define GRAPH_GRID_STARTING_NODE_H

#include <iostream> 
#include <algorithm>
#include <string>
#include <vector> 
#include <cstdlib>
#include <cmath>
#include <queue> 
#include "graph.h" 
#include "scale_projection.h"
#include "graph_grid.h"

using namespace std;

typedef struct GridPair_key {
    //this pair will store node id and distance to T0 as a pair to be used as a key for the hashmap 
    int first; //node id 
    double second; //distance to T0

    bool operator==(const struct GridPair_key other) const { 
        return (first == other.first
            && second == other.second);
    }
} Gpair;


// struct GridPairHash {
//     size_t operator()(const Gpair g) const {
        // using std::size_t;
        // using std::hash;
        // using std::string;
// 
        // return ((hash<int>()(g.first)
                // ^ (hash<double>()(g.second) << 1)) >> 1);
// 
//     }
// };

/* to sort the min priority queue for cloest nodes to T0 */
struct Comp_dist_to_t { 
    bool operator()(const Gpair node1, const Gpair node2) const {
        return node1.second > node2.second ;
    }
};



/* return the distance between 2 nodes are within the required radius */
double dist_from_T0(Point* traj_nd, node g_nd);

/* check whether there are any nodes in the cells included in the given searching range */
bool available_nodes(Grid* grid, int col, int row, int range);

/* expand the included grid cells from n*n to (n+1)*(n+1) */
void add_range_to_Q(Grid* grid, Graph* graph, int col, int row,
int range, Point* traj_nd, priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t>& PQ);

/* check if the outer layer of cells is touched by the radius of distance to peak */
bool range_check(Grid* grid, Point* traj_nd, Graph* graph, priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> PQ);

/* list out the node IDs of the nodes that are within the specified distance using grid look up */
priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> GridSearch(Graph* graph, Grid* grid, Point* traj_nd);


Gpair next_closest_node(Graph* graph, Grid* grid, Point* traj_nd, priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t>& PQ);

vector<Gpair> next_n_nodes(Graph* graph, Grid* grid, Point* traj_nd, 
priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t>& PQ, int n, double radius);

#endif


// class LookUp {
//     private:
        // Point* traj_nd;
// 
        // double lat_min, lat_max, lon_min, lon_max;
        // double g_dist1, g_dist2, g_dist3;
        // double e_dist1, e_dist2;
// 
        // int grid_h, grid_w;
// 
        // double box_height, box_width;
// 
        // double lon_min_to_x, lon_max_to_x, lat_min_to_y, lat_max_to_y; 
        // double x_scale, y_scale;
// 
//     public:
// 
        // /* assign the grid cell a node belongs to, grid_h and grid_w are user defined */
        // vector<int> assign_grid(node nd, int grid_h, int grid_w, double box_height, double box_width, double x_scale, double y_scale);
// 
        // /* find all the nodes in the same grid */
        // vector<int> nodes_in_same_grid(Graph* graph, node traj_nd, int grid_h, int grid_w);
// 
        // /* assign the grid for all the nodes in the graph */
        // void graph_grid(Graph* graph, int grid_h, int grid_w, double box_height, double box_width, double x_scale, double y_scale);
// };
