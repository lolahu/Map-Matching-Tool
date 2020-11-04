#include "scale_projection.h"

double Bounds::degree_to_radian(double a) {
    return a*M_PI/180.0;
}

double Bounds::geodesic_dist(double lat1, double lon1, double lat2, double lon2) {
    double lat_r_1= degree_to_radian(lat1);
    double lon_r_1= degree_to_radian(lon1);
    double lat_r_2= degree_to_radian(lat2);
    double lon_r_2= degree_to_radian(lon2);
    //haversine formula
    double dist=earth_r*acos(cos(lat_r_1)*cos(lat_r_2)*cos(lon_r_1-lon_r_2)+sin(lat_r_1)*sin(lat_r_2));
    return dist;
}

double Euc_distance::lon_mercator_proj(double lon, double lon_min) {
    return (lon-lon_min) * M_PI / 180.0;
}

// need to make an corner case where the area goes through +-180 degree in longitute, maybe long_max - long_min cannot be larger than 180?

double Euc_distance::lat_mercator_proj(double lat, double lat_min) {
    double lat_r = (lat-lat_min) * M_PI / 180.0; 
    double x = log(tan(0.25 * M_PI + 0.5 * lat_r)) - log(tan(0.25 * M_PI));
    return x;
}

double Euc_distance::euc_dist(double lat1, double lon1, double lat2, double lon2) {
    double dist = sqrt(pow((lon2-lon1),2.0)+pow((lat2-lat1),2.0));
    return dist;
}

void Euc_distance::calc_edge_cost(Graph* graph, double lat_scale, double lon_scale) {
    //calculate the cost for every edge in the graph
    for(int i = 0; i < graph -> n_nodes; i++) {
    graph -> nodes[i].lat = lat_mercator_proj(graph -> nodes[i].lat, graph -> min_lat) * lat_scale;
    graph -> nodes[i].longitude = lon_mercator_proj(graph -> nodes[i].longitude, graph -> min_long) * lon_scale;
    }

    // graph -> min_lat = ed.lat_mercator_proj(graph -> min_lat, graph -> min_lat) * lat_scale;
    // graph -> max_lat = ed.lat_mercator_proj(graph -> max_lat, graph -> min_lat) * lat_scale; 
    // graph -> min_long = ed.lon_mercator_proj(graph -> min_long, graph -> min_long) * lon_scale;
    // graph -> max_long = ed.lon_mercator_proj(graph -> max_long, graph -> min_long) * lon_scale;

    for(int i = 0; i < graph -> n_edges; i++) {
        struct node src;
        struct node trgt;
        src = graph -> nodes[graph -> edges[i].srcid];
        trgt = graph -> nodes[graph -> edges[i].trgtid];
        graph -> edges[i].cost = euc_dist(src.lat, src.longitude, trgt.lat, trgt.longitude);
    }
}


void flip_map(Graph* graph){
    if(graph -> max_long - graph -> min_long > 180){
        //fliping the bounding box?
    }


}
