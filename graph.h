#ifndef GRAPH_C_H 
#define GRAPH_C_H

#include <iostream> 
#include <fstream> 
#include <sstream>
#include <string>
#include <vector> 
#include <stack>
#include <algorithm>

using namespace std;

#define MAX_BUFF (100)
#define IGNORE_LINES (5)
#define GRAPH_INIT {0, 0, INT_MAX, INT_MIN, INT_MAX, INT_MIN}
#define DEF_NODE {0, 0, 0, 0}


struct node {
    double longitude;
    double lat;
    int id;
    uint64_t osmid;
};

struct edge {
    int srcid;
    int trgtid;
    int id;
    double cost;
};

typedef struct graph {
    int n_edges;
    int n_nodes;

    double min_lat;
    double max_lat;

    double min_long;
    double max_long;

    double original_min_lat;
    double original_min_long;
    double original_max_lat;
    double original_max_long;

    double lat_scale;
    double lon_scale;

    vector<struct node> nodes;
    vector<struct edge> edges;

    vector<int> out_offsets;
    vector<int> out_off_edges;

    vector<int> in_offsets;
    vector<int> in_off_edges;

} Graph;

/* writes the longitude, latitude of the end points of every edge of a given graph to a given file */
void write_graph(Graph* graph, string file_name);

/* checks and updates the graph's bounding box corners accordingly */
void check_boundaries(double latitude, double longitude, Graph* g);

/* reads a file that contains the graph information and returns a graph with all the necessary attributes set */
void read_file(string file_name, Graph* graph);

void read_processed_graph(string file_name, Graph* graph);

/* used for std::sort() the edges of the graph in order of their source id */
bool compare_outedge(struct edge edge1, struct edge edge2);

/* used for std::sort() the edges of the graph in order of their target id */
bool compare_inedge(struct edge edge1, struct edge edge2);

/* used for std::sort() for the nodes of the graph in ascending order of their id */
bool comp_nodes(struct node n1, struct node n2);

/* generates the out going edge offset array and stores it in graph */
void outedge_offset_array(Graph* graph);

/* generates the in going edge offset array and stores it in graph */
void inedge_offset_array(Graph* graph);

/* returns the number of out going edges for a given node id */
int get_outdeg(Graph* graph, int node_id);

/* returns the edge id for the kth out edge of a given node id */
int get_out_edge(Graph* graph, int node_id, int k);

/* returns the number of in coming edges for a given node id */
int get_indeg(Graph* graph, int node_id);

/* returns the edge id for the kth in edge of a given node id */
int get_in_edge(Graph* graph, int node_id, int k);

/* returns a vector containing the node ids of all the incidents node to a given node */
vector<int> get_incident(Graph* graph, int node_id);

/* returns a vector containing incident node ids for the given node_id in the transpose graph */
vector<int> trans_get_incident(Graph* graph, int node_id);

/* performs a dfs from node 0 in a given graph and returns a visited flags array */
vector<bool> DFS_fwd(Graph* graph);

/* performs a dfs from node 0 in the transpose graph and returns a visited flag array */
vector<bool> DFS_bwd(Graph* graph);

/* return the index of a given node id in the nodes vector of a graph 
    uses binary search -> O(log n) */
int binary_search_node(int node_id, Graph* graph);

/* extracts the strongly connected component of the graph */
void scc_graph(Graph* graph, Graph* scc_graph);

/* outputs a graph to a text file*/
void output_graph(Graph* graph, string file_name, double lat_scale, double lon_scale, double lat_min, double lat_max, double lon_min, double lon_max);

#endif