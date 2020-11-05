// #include "graph.h"
#include "trajectory.h"


void add_point(Trajectory* traj, double longitude, double latitude, int timestamp) {
    Point* point = (Point*) malloc(sizeof(Point));
    point -> longitude = longitude; 
    point -> latitude = latitude;
    point -> timestamp = timestamp;
    // int index = traj -> points.size();
    traj -> points.push_back(point);
    int size = traj -> points.size();
    if(size > 1) {
        //add an edge between the two most recent points
        Tedge* edge = (Tedge*) malloc(sizeof(Tedge));
        edge -> src = traj -> points[size - 2];
        edge -> trg = traj -> points[size - 1];
        traj -> edges.push_back(edge);
    }
}

void read_next_k_bytes(ifstream& file, char* buffer, int k) {
    if(!file.eof()) {
        //read from file if we havent reached end of file 
        memset(buffer, 0, k);
        file.read(buffer, k);
    }
} 

// trajecctory.binTracks file: 

// nPoints traceId subId
// lat lon timestamp
// ...
// lat lon timestamp
// nPoints traceId subId
// ...
void extract_next_trajectory(ifstream& file, int offset, Trajectory* traj, double min_long, double min_lat, double lat_scale, double lon_scale) {
    file.seekg(offset, ios::beg);
    if(file.eof()) {
        return;
    }

    char buffer[4];

    //get the number of sampled points in the trajectory
    read_next_k_bytes(file, buffer, TRAJ_VAL_SIZE);
    traj -> length = *(int*)buffer;

    //get the trace id 
    read_next_k_bytes(file, buffer, TRAJ_VAL_SIZE);
    traj -> traceId = *(uint32_t*)buffer;

    //get the sub id 
    read_next_k_bytes(file, buffer, TRAJ_VAL_SIZE);
    traj -> subId = *(uint32_t*)buffer;

    //get the points 
    double longitude;
    double latitude; 
    int timestamp; 
    Euc_distance ed;

    for(int i = 0; i < traj -> length; i++) {

        read_next_k_bytes(file, buffer, TRAJ_VAL_SIZE);
        latitude = *(int*)buffer;
        latitude /= pow(10, LON_LAT_COMMA_SHIFT);
        /* overwrite the node's latitude in mercator projection */
        // cout<<"latitude before: "<<latitude<<endl;
        latitude = ed.lat_mercator_proj(latitude, min_lat) * lat_scale;
        // cout<<"latitude after: "<<latitude<<endl;
// 
        read_next_k_bytes(file, buffer, TRAJ_VAL_SIZE);
        longitude = *(int*)buffer;
        longitude /= pow(10, LON_LAT_COMMA_SHIFT);
        /* overwrite the node's longitude in mercator projection */
        // cout<<"longitude before: "<<longitude<<endl;
        longitude = ed.lon_mercator_proj(longitude, min_long) * lon_scale;
        // cout<<"longitude after: "<<longitude<<endl;

        read_next_k_bytes(file, buffer, TRAJ_VAL_SIZE);
        timestamp = *(int*)buffer;

        add_point(traj, longitude, latitude, timestamp);
    }
}

vector<Trajectory> read_trajectories(string file_path, int k, double min_long, double min_lat, double lat_scale, double lon_scale) { //extract k trajectories?  //will figure it out later 
    ifstream file;
    file.open(file_path, ios::in | ios::binary);
    file.seekg(0, ios::beg);

    Trajectory traj = DEF_TRAJ;
    vector<Trajectory> trajs;
    for(int i = 0; i < k; i++) {
        if(!file.eof()) {
            int offset = file.tellg();
            extract_next_trajectory(file, offset, &traj, min_long, min_lat, lat_scale, lon_scale);
            trajs.push_back(traj);
            traj = DEF_TRAJ;
        }
        else {
            return trajs;
        }
    }
    file.close();
    return trajs;
}

void write_traj(Trajectory* traj, string file_name){
    ofstream file(file_name);
    for(int i = 0; i < traj -> edges.size(); i++) {
    //x y x y 
        double source_lat = traj -> edges[i] -> src -> latitude;
        double source_lon = traj -> edges[i] -> src -> longitude;
        double target_lat = traj -> edges[i] -> trg -> latitude;
        double target_lon = traj -> edges[i] -> trg -> longitude;
    
        file << source_lon << " " << source_lat << " " << target_lon << " " << target_lat << endl; // what (Vi, Tj) should looks like
    }
    file.close();
}


void cleanup_trajectory(Trajectory* traj) {
    for(int i = 0; i < traj -> points.size(); i++) {
        free(traj -> points[i]);
    }
    for(int i = 0; i < traj -> edges.size(); i++) {
        free(traj -> edges[i]);
    }
}

Euc_distance ed;

 void calc_traj_edge_cost(Trajectory* traj) {
    for(int i = 0; i <  traj -> edges.size(); i++){
        traj -> edges[i] -> cost = ed.euc_dist(traj -> edges[i] -> src -> latitude, traj -> edges[i] -> src -> longitude, traj -> edges[i] -> trg -> latitude, traj -> edges[i] -> trg -> longitude);
    }
    return;
}

// int main() {
//     vector<Trajectory> trajs = read_trajectories("trajectories/saarland-geq50m-clean-unmerged-2016-10-09-saarland.binTracks", 1);
//     Point* cur = trajs[0].head;
//     cout << trajs[0].length << endl;
//     while(cur != NULL) {
//         cout << cur -> longitude << " " << cur -> latitude << endl;
//         cur = cur -> next;
//     }
//     cur = trajs[0].head;
//     Point* next;
//     while(cur != NULL) {
//         next = cur -> next;
//         free(cur);
//         cur = next;
//     }
//     return 0;
// }