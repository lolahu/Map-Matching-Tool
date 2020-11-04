#include "graph.h"
#include "scale_projection.h"

void write_graph(Graph* graph, string file_name) { 
    ofstream file(file_name);
    for(int i = 0; i < graph -> n_edges; i++) {
        //x y x y 
        int source = graph -> edges[i].srcid;
        int target = graph -> edges[i].trgtid;

        file << graph -> nodes[source].longitude << " " << graph -> nodes[source].lat << " " << graph -> nodes[target].longitude << " " << graph -> nodes[target].lat << endl;
    }
    file.close();
}

void check_boundaries(double latitude, double longitude, Graph* g) {
    if(g -> max_lat <= latitude) {
        g -> max_lat = latitude;
    }
    if( g -> min_lat > latitude) {
        g -> min_lat = latitude;
    }
    if(g -> max_long <= longitude) {
        g -> max_long = longitude;
    }
    if(g -> min_long > longitude) {
        g -> min_long = longitude;
    }
}

void read_file(string file_name, Graph* graph) {
    if(file_name.empty()) {
        return;
    }
    ifstream file; 
    file.open(file_name);
    if(!file) {
        cerr << "Unable to open file";
        return;
    }

    string buffer;
    /* skip the first five lines */
    for(int i = 0; i < IGNORE_LINES; i++) {
        getline(file, buffer);
    }
    /* read the total number of nodes and edges, store them in graph struct */
    file >> graph -> n_nodes >> graph -> n_edges;

    /* now read everything
       read line into buffer, scan line number, osmid, lat, long, .. (keep what matters) */
    getline(file, buffer);
    for(int i = 0; i < graph -> n_nodes ; i++) {
        getline(file, buffer);
        istringstream vals(buffer);
        struct node n;
        vals >> n.id >> n.osmid >> n.lat >> n.longitude;
        graph -> nodes.push_back(n);
        check_boundaries(graph -> nodes[i].lat, graph -> nodes[i].longitude, graph);
    }
    for(int i = 0; i < graph -> n_edges; i++) {
        getline(file, buffer);
        istringstream vals(buffer);
        struct edge e;
        vals >> e.srcid >> e.trgtid;
        e.id = i;

        graph -> edges.push_back(e);
    }

    //write a .dat file containing the graph's longitude and latitude coordinates
    write_graph(graph, "graph_lat_lon.dat");   

    file.close();

    //compute offset arrays 
    outedge_offset_array(graph);
    inedge_offset_array(graph);
    
    //write another file containing the projected coordinates of the graph 
    
    return;
}

// void convert_coordinates(Graph* graph, double x_scale, double y_scale){
    // Euc_distance ed;
    // overwrite the node's coordinates in mercator projection
    // for(int i = 0; i < graph -> n_nodes; i++) {
        // graph -> nodes[i].lat = ed.lat_mercator_proj(graph -> nodes[i].lat, graph -> min_lat);
        // graph -> nodes[i].longitude = ed.lon_mercator_proj(graph -> nodes[i].longitude, graph -> min_long);
    // }
    // write_graph(graph, "graph_x_y.dat");
// 
    // return;
    // }


void read_processed_graph(string file_name, Graph* graph) {
    if(file_name.empty()) {
        return;
    }
    ifstream file; 
    file.open(file_name);
    if(!file) {
        cerr << "Unable to open file";
        return;
    }

    string buffer;
    // /* skip the first five lines */
    // for(int i = 0; i < IGNORE_LINES; i++) {
        // getline(file, buffer);
    // }
    /* read the total number of nodes and edges, store them in graph struct */
    file >> graph -> n_nodes >> graph -> n_edges >> graph -> lat_scale >> graph -> lon_scale>>
     graph -> original_min_lat >> graph -> original_max_lat >> graph -> original_min_long >> graph -> original_max_long;

    /* now read everything
       read line into buffer, scan line number, osmid, lat, long, .. (keep what matters) */
    getline(file, buffer);
    for(int i = 0; i < graph -> n_nodes ; i++) {
        getline(file, buffer);
        istringstream vals(buffer);
        struct node n;
        vals >> n.id >> n.osmid >> n.lat >> n.longitude;
        graph -> nodes.push_back(n);
        check_boundaries(graph -> nodes[i].lat, graph -> nodes[i].longitude, graph);
    }
    for(int i = 0; i < graph -> n_edges; i++) {
        getline(file, buffer);
        istringstream vals(buffer);
        struct edge e;
        vals >> e.srcid >> e.trgtid >> e.cost;
        e.id = i;

        graph -> edges.push_back(e);
    }
    write_graph(graph, "process_graph_x_y.dat");

   file.close();

   //compute offset arrays 
   outedge_offset_array(graph);
   inedge_offset_array(graph);
   
   //write another file containing the projected coordinates of the graph 
   
   return;
}


bool compare_outedge(struct edge edge1, struct edge edge2) {
    if(edge1.srcid == edge2.srcid) {
        return edge1.trgtid < edge2.trgtid;
    }
    return edge1.srcid < edge2.srcid;
}

bool compare_inedge(struct edge edge1, struct edge edge2) { //int , check
    if(edge1.trgtid == edge2.trgtid) {
        return edge1.srcid < edge2.srcid;
    }
    return edge1.trgtid < edge2.trgtid;
}

void outedge_offset_array(Graph* graph) {
    vector<struct edge> out_edges = graph -> edges;
    sort(out_edges.begin(), out_edges.end(), compare_outedge);
    vector<int> offset{0};
    int index = 0;
    int k;
    int i;
    for(i = 0; i < out_edges.size(); i = k) {
        for(k = i; k < out_edges.size(); k++) {
            if(out_edges[k].srcid == index) {
                continue;
            }
            else {
                break;
            }
        }
        offset.push_back(k);
        index++;
    }
    int to_add = (graph -> n_nodes + 1) - offset.size();
    for(int j = 0; j < to_add; j++) {
        offset.push_back(i);
    }
    
    graph -> out_offsets = offset;
    for(int i = 0; i < out_edges.size(); i++) {
        graph -> out_off_edges.push_back(out_edges[i].id);
    }
}

void inedge_offset_array(Graph* graph) {
    vector<struct edge> in_edges = graph -> edges;
    sort(in_edges.begin(), in_edges.end(), compare_inedge);
    vector<int> offset{0};
    int index = 0;
    int k;
    int i;
    for(i = 0; i < in_edges.size(); i = k) {
        for(k = i; k < in_edges.size(); k++) {
            if(in_edges[k].trgtid == index) {
                continue;
            }
            else {
                break;
            }
        }
        offset.push_back(k);
        index++;
    }

    int to_add = (graph -> n_nodes + 1) - offset.size();
    for(int j = 0; j < to_add; j++) {
        offset.push_back(i);
    }
    graph -> in_off_edges.clear();
    graph -> in_offsets.clear();

    graph -> in_offsets = offset;
    for(int i = 0; i < in_edges.size(); i++) {
        graph -> in_off_edges.push_back(in_edges[i].id);
    }
}

int get_outdeg(Graph* graph, int node_id) {
    return graph -> out_offsets[node_id + 1] - graph -> out_offsets[node_id];
}

int get_out_edge(Graph* graph, int node_id, int k) {
    int neighbours = get_outdeg(graph, node_id);
    if(k >= neighbours) {
        //invalid
        return -1;
    }

    int index = graph -> out_offsets[node_id] + k;
    return graph -> out_off_edges[index];
}

int get_indeg(Graph* graph, int node_id) {
    return graph -> in_offsets[node_id + 1] - graph -> in_offsets[node_id];
}

int get_in_edge(Graph* graph, int node_id, int k) {
    int neighbours = get_indeg(graph, node_id);
    if(k >= neighbours) {
        //invalid
        return -1;
    }

    int index = graph -> in_offsets[node_id] + k;
    return graph -> in_off_edges[index];
}

vector<int> get_incident(Graph* graph, int node_id) {
    vector<int> incidents;

    int n_neighbours = get_outdeg(graph, node_id);
    int index = graph -> out_offsets[node_id];

    for(int i = index; i < (index + n_neighbours); i++) {
        int edge_id = graph -> out_off_edges[i];
        int neighbour_id = graph -> edges[edge_id].trgtid;
        incidents.push_back(neighbour_id);
    }

    return incidents;
}

vector<int> trans_get_incident(Graph* graph, int node_id) {
    vector<int> incidents;

    int n_neighbours = graph -> in_offsets[node_id + 1] - graph -> in_offsets[node_id];
    int index = graph -> in_offsets[node_id];

    for(int i = index; i < index + n_neighbours; i++) {
        int edge_id = graph -> in_off_edges[i];
        int neighbour_id = graph -> edges[edge_id].srcid;
        incidents.push_back(neighbour_id);
    }
    return incidents;
}

vector<bool> DFS_fwd(Graph* graph) {
    vector<bool> visited_fwd(graph -> n_nodes, false);
    stack<int> Stack;
    //start node is the first node in the graph
    int node_id = 0;
    Stack.push(node_id);
    while(!Stack.empty()) {
        node_id = Stack.top();
        Stack.pop();

        if(!visited_fwd[node_id]) {
            visited_fwd[node_id] = true;
        }

        vector<int> incidents = get_incident(graph, node_id);
        for(int i = incidents.size() - 1; i >= 0; i--) {
            if(!visited_fwd[incidents[i]]){
                Stack.push(incidents[i]);
            }
        }
    }
    return visited_fwd;
}

vector<bool> DFS_bwd(Graph* graph) {
    vector<bool> visited_bwd(graph -> n_nodes, false);
    stack<int> Stack;
    int node_id = 0;
    Stack.push(node_id);
    while(!Stack.empty()) {
        node_id = Stack.top();
        Stack.pop();
        if(!visited_bwd[node_id]) {
            visited_bwd[node_id] = true;
        }
        vector<int> incidents = trans_get_incident(graph, node_id);
        for(int i = incidents.size() - 1; i >= 0; i--) {
            if(!visited_bwd[incidents[i]]){
                Stack.push(incidents[i]);
            }
        }
    }
    return visited_bwd;
}

bool comp_nodes(struct node n1, struct node n2) {
    return n1.id < n2.id;
}

int binary_search_node(int node_id, Graph* graph) {
    int lower = 0; 
    int higher;
    if(node_id >= graph -> nodes.size()) {
        higher = graph -> nodes.size() - 1;
    }
    else {
        higher = node_id;
    }
    while(lower <= higher) {
        int mid_point = lower + (higher - lower) / 2;
        if(graph -> nodes[mid_point].id == node_id) {
            return mid_point;
        }
        else if(graph -> nodes[mid_point].id < node_id) {
            //iterate to the right 
            lower = mid_point + 1;
        }
        else {
            //iterate to the left  
            higher = mid_point - 1;
        }
    }
    //not found
    return -1;

}

void scc_graph(Graph* graph, Graph* SCC_graph) {
    vector<bool> visited_fwd = DFS_fwd(graph);
    vector<bool> visited_bwd = DFS_bwd(graph);

    //check for any nodes that have both their flags checked
    for(int i = 0; i < graph -> n_nodes; i++) {
        if(visited_fwd[i] && visited_bwd[i]) {
            SCC_graph -> nodes.push_back(graph -> nodes[i]);
            SCC_graph -> n_nodes += 1;
        }
    }
    sort(SCC_graph -> nodes.begin(), SCC_graph -> nodes.end(), comp_nodes); //for binary search

    //now add all the edges 
    for(int i = 0; i < graph -> n_edges; i++) {
        //get the end points for each edge and see if they are in the scc graph
        int src = graph -> edges[i].srcid;
        int trgt = graph -> edges[i].trgtid;
        if(visited_fwd[src] && visited_bwd[src] && visited_bwd[trgt] && visited_bwd[trgt]) {
            SCC_graph -> edges.push_back(graph -> edges[i]);
            SCC_graph -> edges.back().id = SCC_graph -> n_edges;
            SCC_graph -> n_edges += 1;
        }
    }

    //parallelize this?
    for(int i = 0; i < SCC_graph -> n_edges; i++) {
        int source = SCC_graph -> edges[i].srcid;
        int target = SCC_graph -> edges[i].trgtid;
        int indx = binary_search_node(source, SCC_graph);
        SCC_graph -> edges[i].srcid = indx;
        indx = binary_search_node(target, SCC_graph);
        SCC_graph -> edges[i].trgtid = indx;
    }
    for(int i = 0; i < SCC_graph -> n_nodes; i++) {
        SCC_graph -> nodes[i].id = i;
    }
    write_graph(SCC_graph, "SCC_graph.dat");

    //compute the inedge and outedge offsets for the graph
    inedge_offset_array(SCC_graph);
    outedge_offset_array(SCC_graph);
}

void output_graph(Graph* graph, string file_name, double lat_scale, double lon_scale, 
double lat_min, double lat_max, double lon_min, double lon_max) {
    vector<struct node> all_nodes = graph -> nodes;
    vector<struct edge> all_edges = graph -> edges;

    ofstream txt_file(file_name);
    txt_file << all_nodes.size() << endl;
    txt_file << all_edges.size() << endl;
    txt_file << lat_scale << endl;
    txt_file << lon_scale << endl;
    txt_file << lat_min << endl;
    txt_file << lat_max << endl;
    txt_file << lon_min << endl;
    txt_file << lon_max << endl;


    for(int i = 0; i < all_nodes.size(); i++) {
        txt_file << all_nodes[i].id << " " << all_nodes[i].osmid << " " << all_nodes[i].lat << " " << all_nodes[i].longitude << endl;
    }

    for(int i = 0; i < all_edges.size(); i++) {
        txt_file << all_edges[i].srcid << " " << all_edges[i].trgtid << " " << all_edges[i].cost << endl;
    }

    txt_file.close();
    return;
}