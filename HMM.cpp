#include "HMM.h"

vector<Gpair> candidates(Graph* graph, Grid* grid, Point* traj_nd, int n){
    grid -> curr_range = 0;
    priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> dist_PQ  = GridSearch(graph, grid, traj_nd);
    vector<Gpair> next_n = next_n_nodes(graph, grid, traj_nd, dist_PQ, n); //in ascending order by the distance to trajectory node;
    return next_n;
}

// Gpair cloest_can(Graph* graph, Grid* grid, Point* traj_nd){
    // grid -> curr_range = 0;
    // priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> dist_PQ  = GridSearch(graph, grid, traj_nd);
    // Gpair close = dist_PQ.top();
    // return close;
// }
// 
// stack<int> path_closest_can(Graph* graph, Grid* grid, Trajectory* traj){
    // stack<int> path;
    // for (int i = 0; i < traj ->length; i++){
        // Gpair best_can = cloest_can(graph, grid, traj -> points[i]);
        // path.push(best_can.first);
        // cout<<"node ID: "<<best_can.first<<" distance "<<best_can.second<<endl;
// 
    // }
    // return path;
// }

double emission(double sigma, double dist){ //change the dist to a T and a candidate and calculate the dist within the function
    double prob = 1/ (sqrt(2 * M_PI) * sigma) * exp( - 0.5 * pow((dist/sigma),2.0));
    return prob;
}
// 
double transition(double beta, Point*  T1, Point* T2, double SP){
    double dist = sqrt(pow((T2 -> longitude - T1 -> longitude),2.0)+pow((T2 -> latitude - T1 -> latitude),2.0));
    double prob = 1/beta * exp(-abs(SP - dist)/beta);
    return prob;
}


double sigma_est(Graph* graph, Grid* grid, Trajectory* traj){
    vector<double> error_array;
    for (int i = 0; i < traj -> length; i++){
        grid -> curr_range = 0;
        priority_queue<Gpair, vector<Gpair>, Comp_dist_to_t> PQ  = GridSearch(graph, grid, traj -> points[i]);
        /* the second component in the pair is the euclidean distance of the current vertex to the current trajectory point */
        if(!PQ.empty()){
            Gpair p = PQ.top();
            error_array.push_back(p.second);
        }
    }
    /* finding the median of the error_array */
    double med = getMedian(error_array);

    double sigma = 1.4826 * med; /* based on the formula proposed by Newson and Krumm in 2009 */
    return sigma;
}

double getMedian(vector<double> array){
    int n = array.size();
    // sort the array
    sort(array.begin(), array.end());
    // check for even case
    if (n % 2 != 0){
        return array[n / 2];
        }
    return (array[(n - 1) / 2] + array[n / 2]) / 2.0;
}
bool compare_pair_dist(pair<int, double> pair1, pair<int, double> pair2) {
    return pair1.second < pair2.second;
}


vector<pair<int, double>>  emission_set(Graph* graph, Grid* grid, Point* traj_nd, int n, double sigma){ //with candidate node ids
    grid -> curr_range = 0;
    vector<Gpair> next_n = candidates(graph, grid, traj_nd, n); // in ascending order by the distance to trajectory node;
    // /* priority_queue<pair<int, double> , vector<pair<int, double>>, Comp_cdd_emi> emission_PQ; */
    vector<pair<int, double>> emissions;
    double sum_emi =0;
    for (int i = 0; i < n; i++){
        double dist = next_n[i].second;
        double emi_prob = emission(sigma, dist);
        pair<int, double> emi_pair; // sorted by the V's distance to T, in increasing order;
        emi_pair.first = next_n[i].first;
        emi_pair.second = emi_prob;
        // cout<<"emi_pair.first: "<<emi_pair.first<<" emi_pair.second "<<emi_pair.second<<endl;
        emissions.push_back(emi_pair);
        sum_emi += emi_prob;
    }
    // normalization for the probabilities
    for (int i =0; i < emissions.size();i++){
        emissions[i].second = emissions[i].second/sum_emi;
        // cout<<"emission probability after normalization: "<<emissions[i].second<<endl;
    }
    return emissions; // sorted by the V's distance to T, in increasing order;
}


vector<pair<int,double>> get_inv_incident_pair(Graph* graph, int node_id){
    vector<pair<int,double>> incidents;

    int n_neighbours = graph -> in_offsets[node_id + 1] - graph -> in_offsets[node_id];
    int index = graph -> in_offsets[node_id];

    for(int i = index; i < index + n_neighbours; i++) {
        int edge_id = graph -> in_off_edges[i];
        int neighbour_id = graph -> edges[edge_id].srcid;
        pair<int,double> p;
        p.first = neighbour_id;
        p.second = graph -> edges[edge_id].cost;
        incidents.push_back(p);
    }
    return incidents;
}


int src_candidate(Graph* graph, vector<Gpair> candidates){
    int num_tar = 0;
    for (int i = 0; i < candidates.size(); i++){
        Gpair p = candidates[i];
        graph -> nodes[p.first].target =true;
        // cout<<"flagging: "<<graph -> nodes[p.first].id<<" "<<graph -> nodes[p.first].target<<endl;
        num_tar++;
    }
    return num_tar;
}


double beta_est(double alpha, double t, double radius){
    double b = alpha/(1.0 - alpha) * radius/ t; 
    // d is the search radius used to identify candidate roads around the observations zi.
    // where t is the maximal ratio between great circle distance and route distance which can be considered plausible.
    // The calibration parameter β can be calculated intuitively as
    return b;
}

vector<pair<int, double>> tran_dijkstra(Graph* graph, int node_id, vector<Gpair> prev_candidates){ // run dijkstra for transition probability calculation
    // cout<<"is the source node a target?: "<<graph -> nodes[node_id].id<<" "<<graph -> nodes[node_id].target<<endl;
    /* all distance set to infinity while reading the graph */
    int num_tar = src_candidate(graph, prev_candidates);

    // cout<<"num_tar: "<<num_tar<<endl;
    priority_queue<struct node, vector<struct node>, comp_travel_cost> PQ;
    vector <int> dirty_nodes; 
    graph -> nodes[node_id].dist = 0; 
    struct node src_nd = graph -> nodes[node_id]; // making a copy of the graph node
    // cout<<"is the source node a target now?: "<<src_nd.id<<" "<<src_nd.target<<endl;

    PQ.push(src_nd);
    dirty_nodes.push_back(node_id);
    // cout<<"is the source node a target nowwww?: "<<PQ.top().id<<" "<<PQ.top().dist<<" target?: "<<PQ.top().target<<endl;
    while(!PQ.empty()){
        struct node nd = PQ.top();
        // cout<<"poped a node from PQ: id: "<<nd.id<<endl;
        PQ.pop(); 
        if(nd.settled) { 
            continue;
        }
        nd.settled = true;
        if (nd.target == true){
            nd.target = false;
            num_tar--;
            // cout<<"---------hit one previous candidate: "<<nd.id<<endl;
        }
        if(num_tar == 0){
            break;
        }
        vector<pair<int,double>> incidents = get_inv_incident_pair(graph, nd.id); //
        for (pair<int,double> adj : incidents){
            int id = adj.first;
            double distance = adj.second;
            // cout<<"src node: "<<nd.id<<" trg node: "<<id<<" distance between: "<<distance<<" trg node current dist: "<<graph -> nodes[id].dist<<endl;
            if (graph -> nodes[adj.first].dist > nd.dist + adj.second){
                // cout<<"src node: "<<nd.id<<" node and its original dist before update: "<<adj.first<<" "<<graph -> nodes[adj.first].dist<<endl;
                graph -> nodes[adj.first].dist = nd.dist + adj.second;
                // cout<<"src node: "<<nd.id<<" after update: "<<adj.first<<" "<<graph -> nodes[adj.first].dist<<endl;
                graph -> nodes[adj.first].parent_id = nd.id;
                PQ.push(graph -> nodes[adj.first]);
                dirty_nodes.push_back(graph -> nodes[adj.first].id);
                // cout<<"PQ.size(): "<<PQ.size()<<" first one: "<<PQ.top().id<<" "<<PQ.top().dist<<endl;
            }
        }
    }
    // extracting the SP from each previous candidate node to this current candidate node
    vector<pair<int, double>> SP;
    for (int i = 0; i < prev_candidates.size(); i++){
        Gpair can = prev_candidates[i];
        SP.push_back(make_pair(graph -> nodes[can.first].id,graph -> nodes[can.first].dist));
    }
    // reset everything touch was touch in this dijkstra for the next run.
    // cout<<"number of dirty_nodes: "<<dirty_nodes.size()<<endl;
    for (int i = 0; i < dirty_nodes.size(); i++){
        graph -> nodes[dirty_nodes[i]].dist = INFINITY;
        graph -> nodes[dirty_nodes[i]].target = false;
        graph -> nodes[dirty_nodes[i]].settled = false;
        graph -> nodes[dirty_nodes[i]].parent_id = -1;
    }
    return SP;
}

State state_prob(Graph* graph, Grid* grid, Point* T1, Point* T2, double beta, double sigma, 
int n, State prev_state, vector<pair<int, double>> emi_probs){ 
    // emission vector sorted by the V's distance to T, in increasing order;
    vector<Gpair> curr_candidates = candidates(graph, grid, T2, n); // don't forget trans matric normalization
    vector<Gpair> prev_candidates = candidates(graph, grid, T1, n);

    // vector<pair<int, double>> emi_set = emission_set(graph, grid, T2, n, sigma); this is given in the parameters

    State curr_state;
    curr_state.cdd_nd_id.resize(n, 0.0);
    curr_state.prdc_state.resize(n, -1);
    curr_state.state_prob.resize(n, 0.0);

    for (int i = 0; i < curr_candidates.size(); i++){
        Gpair gp = curr_candidates[i]; 
        curr_state.cdd_nd_id[i] = gp.first;
        vector<pair<int, double>> trans_vec = tran_dijkstra(graph, gp.first, prev_candidates);
        /* max{P(i,j)*T(k,j)*Q(i-1,k)} for each P(i,j)/Q(i,j)/candidate cell */
        for (int j = 0; j < trans_vec.size(); j++){
            pair<int, double> SP = trans_vec[j];
            double trans_prob = transition(beta, T1, T2, SP.second);
            // cout<<"trans_prob: "<<trans_prob<<endl;
            double pair_prob = prev_state.state_prob[i] * trans_prob * emi_probs[i].second;// picking the maximum so the constant emission prob isn't neccesary
            /* prev_state.state_prob[i] * trans_prob * emi_probs[i].second; */
            // cout<<"pair_prob: "<<pair_prob<<endl;
            // cout<<"curr_state.state_prob "<<i<<" "<<curr_state.state_prob[i]<<endl;
            if( pair_prob > curr_state.state_prob[i]){
                cout<<"max state prob got updated from: "<<curr_state.state_prob[i]<<" to ";
                curr_state.state_prob[i] = pair_prob; // curr_state and curr_candidates has the same length, use the same index i
                curr_state.prdc_state[i] = SP.first;
                cout<<curr_state.state_prob[i]<<" from node: "<<curr_state.prdc_state[i] <<endl;
            }
        }  
    }
    /* normalize the state probability */
    double sum_state_prob = 0.0;// std::accumulate(curr_state.state_prob.begin(), curr_state.state_prob.end(), 0);
    for (int i = 0; i < curr_state.state_prob.size(); i++){
        sum_state_prob += curr_state.state_prob[i];
        // cout<<"curr_state.state_prob "<<i<<" "<<curr_state.state_prob[i]<<endl;
    }
    cout<<"sum_state_prob: "<<sum_state_prob<<endl;
    // calculatet the sum in a loop so can also figure out what's the max state probablitie and flag the predecessor 
    // -- need to flag predecessor for each state cel not just the max!!!!
    for (int i = 0; i < curr_state.state_prob.size(); i++){
        curr_state.state_prob[i] = curr_state.state_prob[i]/sum_state_prob;
        cout<<"candidate ID: "<<curr_state.cdd_nd_id[i]<<" state_prob after normalization: "<<i<<" "<<curr_state.state_prob[i]<<" predecessor: "<<curr_state.prdc_state[i]<<endl;
    }
    return curr_state;
}


State create_state0(vector<pair<int, double>>  emi_set){
    State state_0; // does it need initializatoin?
    state_0.cdd_nd_id.resize(10, 0.0);
    state_0.prdc_state.resize(10, -1);
    state_0.state_prob.resize(10, 0.0);

    for (int i = 0; i < emi_set.size(); i++){
     state_0.cdd_nd_id[i] = emi_set[i].first ;
     
    }
    for (int i = 0; i < emi_set.size(); i++){
     state_0.state_prob[i] = emi_set[i].second;
    }

    return state_0;
}


vector<int> best_path(Graph* graph, Grid* grid, Trajectory* traj, int n, double sigma, double beta ){ 
    
    vector<pair<int, double>> emi_0 = emission_set(graph, grid, traj->points[0], n, sigma);
    State state0 = create_state0(emi_0);
    stack<State> state_stack; // want last in first out -- stack
    state_stack.push(state0);

    State prev_state = state0; 

    for (int i =  0; i < 18; i++){ // change back to i < traj -> length - 1
        State prev_state = state_stack.top();
        vector<pair<int, double>> curr_emi = emission_set(graph, grid, traj->points[i + 1], n, sigma);
        State cur_state = state_prob(graph, grid, traj->points[i], traj->points[i + 1], beta, sigma, n, prev_state, curr_emi);
        state_stack.push(cur_state);
        prev_state = cur_state;

    }
    vector<int> best_path; 
    while(!state_stack.empty()){
        State curr = state_stack.top();
        state_stack.pop();
        double max_prob = 0.0;
        int pre_can = 0;
        for (int i = 0; i < curr.prdc_state.size(); i++){
            if( curr.state_prob[i] > max_prob ){
                pre_can = curr.prdc_state[i];
            }
        }
    best_path.push_back(pre_can); // remove nodes that have the same node ID if they're consective
    }
    reverse(best_path.begin(), best_path.end());
    /* stack is LIFO, so the match at the traj end is the first one poped, 
    we need to reverse the order, so that the last item pushed 
    from best path is traj[0] match and will be index 0 in the returned path*/

    for (int i = 0; i < best_path.size() - 2; i++){
        if(best_path[i] == best_path[i+1] || best_path[i] == -1){
        best_path.erase(best_path.begin() + i);
        }
    }
    return best_path;
}

stack<int> node_to_node_dijkstra(Graph* graph, int node_id, int node_id2){ 
    // cout<<"run dijkstra for final nodes\n";
    priority_queue<struct node, vector<struct node>, comp_travel_cost> PQ;
    vector <int> dirty_nodes; 
    graph -> nodes[node_id].dist = 0; 
    struct node src_nd = graph -> nodes[node_id]; 

    PQ.push(src_nd);
    dirty_nodes.push_back(node_id);
    while(!PQ.empty()){
        struct node nd = PQ.top();
        PQ.pop(); 
        if(nd.id == node_id2){
            break;
        }
        if(nd.settled) { 
            continue;
        }
        nd.settled = true;
        vector<pair<int,double>> incidents = get_inv_incident_pair(graph, nd.id); //
        for (pair<int,double> adj : incidents){
            int id = adj.first;
            double distance = adj.second;
            if (graph -> nodes[adj.first].dist > nd.dist + adj.second){
                graph -> nodes[adj.first].dist = nd.dist + adj.second;
                graph -> nodes[adj.first].parent_id = nd.id;
                PQ.push(graph -> nodes[adj.first]);
                dirty_nodes.push_back(graph -> nodes[adj.first].id);
            }
        }
    }
    stack<int> SP; // Stack: Last In First Out (LIFO)
    int curr_nd = node_id2;
    while(curr_nd != node_id){
        SP.push(curr_nd);
        curr_nd = graph -> nodes[curr_nd].parent_id;
        // cout<<"curr_nd: "<<curr_nd<<endl;
    }
    // reset everything touch was touch in this dijkstra for the next run.
    for (int i = 0; i < dirty_nodes.size(); i++){
        graph -> nodes[dirty_nodes[i]].dist = INFINITY;
        graph -> nodes[dirty_nodes[i]].target = false;
        graph -> nodes[dirty_nodes[i]].settled = false;
        graph -> nodes[dirty_nodes[i]].parent_id = -1;
    }
    return SP;
}

vector<int> best_path_dijkstra(Graph* graph, vector<int> best_path){
    // vector<int> best = best_path(&after_graph, &grid, &traj, 10, sigma, 40);
    vector<int> complete_path;
    stack<int> SP;
    for (int i = 0; i < best_path.size() - 2; i++){
        int id = best_path[i];
        int id2 = best_path[i+1];
        SP = node_to_node_dijkstra(graph, id, id2);
        while(!SP.empty()){
            complete_path.push_back(SP.top());
            SP.pop();
        }
    }
    return complete_path;
}


void write_HMM_graph(Graph* graph, vector<int> complete_path, string file_name){ 
    ofstream file(file_name);
    for (int i =  0; i < complete_path.size() - 2; i++){
        int id = complete_path[i];
        int id2 = complete_path[i+1];
        double src_lat, src_lon, trg_lat, trg_lon;
        src_lat = graph -> nodes[id].lat;
        src_lon = graph -> nodes[id].longitude;
        trg_lat = graph -> nodes[id2].lat;
        trg_lon = graph -> nodes[id2].longitude;
        // cout<<i<<" "<<id<<" "<<id2<<" "<<src_lon<< " " << src_lat << " " << trg_lon << " " << trg_lat << endl;
        file << src_lon<< " " << src_lat << " " << trg_lon << " " << trg_lat << endl;
    }
    file.close();
}