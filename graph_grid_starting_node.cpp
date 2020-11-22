#include "graph_grid_starting_node.h"
#include "graph_grid.h"
#include <numeric> 

using namespace std;

double dist_from_T0(Point* traj_nd, node g_nd) {
    double dist; 
    dist = sqrt(pow((traj_nd -> latitude - g_nd.lat), 2) + 
                    pow((traj_nd -> longitude - g_nd.longitude), 2));
    return dist; 
}  

void add_range_to_Q(Grid* grid, Graph* graph, int col, int row, int range, Point* traj_nd, priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t>& PQ){

    // TODO: this is wrong when a cell on the boundary was already added to the PQ
    // Needs to be done separately per loop: hor-top, hor-bot,...
    int left  = max(0, col - range);
    int right = min(grid -> num_columns - 1, col + range);
    int upper = min(grid -> num_rows - 1, row + range);
    int lower = max(0, row - range);

    int starting_idx, ending_idx;
    int num_columns = grid -> num_columns;
    int num_rows    = grid -> num_rows;
//   cout<<"starting to add nodes\n";

   if (grid -> num_rows - 1 >= row + range){
    //    cout<<"entered horizontal top: \n";
        /* horizontal top */
        starting_idx = num_columns * upper + left;
        ending_idx   = num_columns * upper + right;

        int pos1 = grid -> cell_offset[starting_idx];
        int pos2 = grid -> cell_offset[ending_idx + 1]; 
        for (int k = pos1; k < pos2; k++){
                int nd_id = grid -> cell_nodes_list[k];
                Gpair grid_nd;
                grid_nd.first  = nd_id;
                grid_nd.second = dist_from_T0(traj_nd, graph -> nodes[nd_id]);
                PQ.push(grid_nd);
            }
   }
//    cout<<"horizontal top fine\n";
    
    if(upper > lower){
    /* horizontal bottom */
        if (row >= range){
            // cout<<"entered horizontal bottom: \n";
            starting_idx = num_columns * lower + left;
            ending_idx   = num_columns * lower + right;

            int pos1 = grid -> cell_offset[starting_idx];
            int pos2 = grid -> cell_offset[ending_idx + 1]; 
            for (int k = pos1; k < pos2; k++){
                int nd_id = grid -> cell_nodes_list[k];
                Gpair grid_nd;
                grid_nd.first  = nd_id;
                grid_nd.second = dist_from_T0(traj_nd, graph -> nodes[nd_id]);
                PQ.push(grid_nd);
                }
        }
        //  cout<<"horizontal bottom fine\n";

    int index_i;
    if (col >= range){
    /* vertical left */   
        for (int i =  row - range + 1; i <=  min(row + range - 1, num_rows - 1); i++){
            // cout<<"current range: "<<range<<"  i:  "<<i<<endl;
            int a = row - range + 1;
            int b = row + range - 1;
            // cout<<" for vertical left ---- row - range + 1: "<< a << " row + range - 1: " <<b<<endl;
            index_i = (num_columns * i)  + left;
            int pos1 = grid -> cell_offset[index_i];
            int pos2 = grid -> cell_offset[index_i + 1]; 
            // cout<<"index_i: "<<index_i<<" pos1: "<<pos1<<" pos2: "<<pos2<<endl;
                for (int k = pos1; k < pos2; k++){
                    int nd_id = grid -> cell_nodes_list[k];
                    Gpair grid_nd;
                    grid_nd.first = nd_id;
                    grid_nd.second = dist_from_T0(traj_nd, graph -> nodes[nd_id]);
                    PQ.push(grid_nd);
            }}
        }
        //  cout<<"vertical left fine\n";

        if (grid -> num_columns - 1 >= col + range){
            // cout<<"grid -> num_columns - 1 >= col + range is okay\n";
            // cout<<"entered vertical right: \n";
            /* vertical right */
            int a = row - range + 1;
            int b = row + range - 1;
            // cout<<" for vertical right ---- row - range + 1: "<< a << " row + range - 1: " <<b<<endl;
            for (int i =  max(row - range + 1, 0); i <=  row + range - 1; i++){
                index_i = (num_columns * i)  + right;
                int pos1 = grid -> cell_offset[i];
                int pos2 = grid -> cell_offset[i+ 1]; 
                    for (int k = pos1; k < pos2; k++){
                        int nd_id = grid -> cell_nodes_list[k];
                        Gpair grid_nd;
                        grid_nd.first = nd_id;
                        grid_nd.second = dist_from_T0(traj_nd, graph -> nodes[nd_id]);
                        PQ.push(grid_nd);

                }}
        }
        // cout<<"vertical right fine\n";
            }
            
    return;
}

bool range_check(Grid* grid, Point* traj_nd, Graph* graph, priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> PQ){
    double graph_max_x = graph -> max_long;
    double graph_max_y = graph -> max_lat;

    double dist_peak = grid -> dist_to_peak;
    int col = floor(traj_nd -> longitude/ grid -> size);
    int row = floor(traj_nd -> latitude/ grid -> size);
    double T0_x = traj_nd -> longitude;
    double T0_y = traj_nd -> latitude;

    if (PQ.empty()){
         while(PQ.empty() && grid -> curr_range <= max(grid -> num_columns - 1, grid -> num_rows - 1)){
         add_range_to_Q(grid, graph, col, row, grid -> curr_range, traj_nd, PQ);
         grid -> curr_range++;
     }
     add_range_to_Q(grid, graph, col, row, grid -> curr_range, traj_nd, PQ);
    }

    bool within_range = true;

    double left_limit = max(0.00, T0_x - dist_peak);
    int left = max(0, col - grid -> curr_range) + 1;
    double left_bd = left * grid -> size;

    double right_limit = min(graph_max_x, T0_x + dist_peak);
    int right = min(grid -> num_columns - 1, col + grid -> curr_range);
    double right_bd = right * grid -> size;

    double upper_limit = min(graph_max_y, T0_y + dist_peak);
    int upper = min(grid -> num_rows - 1, row + grid -> curr_range);
    double upper_bd = upper * grid -> size;
    
    double lower_limit = max(0.0, T0_y - dist_peak);
    int lower = max(0, row - grid -> curr_range) + 1;
    double lower_bd = lower * grid -> size;

    if (left_limit <= left_bd || right_limit >= right_bd || upper_limit >= upper_bd || lower_limit <= lower_bd){ 
    within_range = false;
    }
    return within_range;
}

priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> GridSearch(Graph* graph, Grid* grid, Point* traj_nd){
    priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> PQ;
    int col = floor(traj_nd -> longitude/ grid -> size);
    int row = floor(traj_nd -> latitude/ grid -> size);

    // cout<<"grid number of columns: "<<grid->num_columns<<" number rows: "<<grid->num_rows<<endl;
    // cout<<"search query point   column: "<<col<<" row: "<<row<<endl;

    while(PQ.empty() && grid -> curr_range <= max(grid -> num_columns - 1, grid -> num_rows - 1)){
        add_range_to_Q(grid, graph, col, row, grid -> curr_range, traj_nd, PQ);
        grid -> curr_range++; 
    }
    add_range_to_Q(grid, graph, col, row, grid -> curr_range, traj_nd, PQ);

    // cout<<" current nodes_idx_list size before range check: "<<PQ.size()<<endl;

    grid -> dist_to_peak = PQ.top().second; 

    bool enough_range = range_check(grid, traj_nd, graph, PQ);
    if (!enough_range) {
        grid -> curr_range++;
        add_range_to_Q(grid, graph, col, row, grid -> curr_range,traj_nd, PQ);
        return PQ;
    }
    else{
        return PQ;
    }
}


Gpair next_closest_node(Graph* graph, Grid* grid, Point* traj_nd, priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t>& PQ){ 
    int col = floor(traj_nd -> longitude/ grid -> size);
    int row = floor(traj_nd -> latitude/ grid -> size);

    Gpair closest_nd = PQ.top();
    PQ.pop();
    grid -> dist_to_peak = PQ.top().second; 
    bool enough_range = range_check(grid, traj_nd, graph, PQ);
    if (!enough_range) {
        grid -> curr_range++;
        add_range_to_Q(grid, graph, col, row, grid -> curr_range, traj_nd, PQ);
    }
    return closest_nd;
}

vector<Gpair> next_n_nodes(Graph* graph, Grid* grid, Point* traj_nd, 
priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t>& PQ, int n, double radius){ 
    int col = floor(traj_nd -> longitude/ grid -> size);
    int row = floor(traj_nd -> latitude/ grid -> size);

cout<<"before increasing range PQ.size(): "<<PQ.size()<<endl;

    while (PQ.size()< n){
        cout<<"increasing the range\n";
        grid -> curr_range++;
        add_range_to_Q(grid, graph, col, row, grid -> curr_range, traj_nd, PQ);
        // /* bool enough_range = range_check(grid, traj_nd, graph, PQ);
        // if (!enough_range) {
            // grid -> curr_range++;
            // add_range_to_Q(grid, graph, col, row, grid -> curr_range, traj_nd, PQ);
        // }*/
    }

    vector<Gpair> next_n;
    for (int i = 0; i < n && PQ.top().second <= radius; i++){
        Gpair g = PQ.top();
        next_n.push_back(g);
        PQ.pop();
    }
    
    //  grid -> curr_range = 0; reset the range inside a function whenever search for a new node;

    return next_n; // in ascending order by the distance to trajectory node;
}



