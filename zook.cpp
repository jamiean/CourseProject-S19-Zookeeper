//  EECS-281 Spring 2018 Project 4
//  zook.cpp
//
//  JunCheng An; jamiean
//
//  Created by JunCheng An on 6/8/18.
//  Copyright © 2018 JunCheng An. All rights reserved.
//

#include "zook.h"


// Required: argv and argv are valid command line information
// Effect  : read command line input, see what mode is on
//           -h    : helper function called, throw HelpFunctionCalled{};
//           -m XX : calculate mode
//                   XX -> MST: turn MST_on to TRUE
//                   XX -> OPTTSP : turn OPT_on to TRUE
//                   XX -> FASTTSP: turn FAST_on to TRUE
//                   XX -> otherwise, throw ModeUnfound{}
void zookeeper::read_command(int argc, char* argv[]) {
    int command = 0;
    int option_ind = 0;
    option long_option[] = {
        {"mode", required_argument, nullptr, 'm'},
        {"help", no_argument, nullptr, 'h'},
        { nullptr, 0, nullptr, '\0' }
    };
    
    while ((command = getopt_long(argc, argv, "m:h",
                                  long_option, &option_ind)) != -1) {
        switch (command) {
            case 'h': {
                std::cout << "Helper function should appear here lol!" << '\n';
                throw HelpFunctionCalled();
                break;
            }
            case 'm': {
                std::string arg = std::string(optarg);
                if (arg == "MST") MST_on = true;
                else if (arg == "OPTTSP") OPT_on = true;
                else if (arg == "FASTTSP") FAST_on = true;
                else throw ModeUnfound();
                break;
            }
            default:
                break;
        }
    }
}



// Modifier: cin
// Effect  : read in map information from standard input
//           if there're cages in both safe and wild area
//              but no cage in intersection area in MST mode
//           ---> throw Invalidcage{};
//           MST       --> read in points_MST
//           FAST/OPT  --> read in points_TSP
void zookeeper::read_map() {
    int int_in = 0;
    
    // Start read in maps integers
    std::cin >> int_in;
    
    if (MST_on) {
        bool safe_ap = false;
        bool wild_ap = false;
        bool inter_ap = false;
        point_MST point_in;
        points_MST.resize(int_in);
        for (size_t i = 0; i < points_MST.size(); ++ i) {
            std::cin >> int_in;
            point_in.x = int_in;
            std::cin >> int_in;
            point_in.y = int_in;
            if (point_in.x > 0 || point_in.y > 0) {
                point_in.type = 's';
                safe_ap = true;
            }
            else if (point_in.x == 0 || point_in.y == 0) {
                point_in.type = 'i';
                inter_ap = true;
            }
            else {
                point_in.type = 'w';
                wild_ap = true;
            }
            points_MST[i] = point_in;
        }
        if (safe_ap && wild_ap && !inter_ap) throw Invalidcage{};
        num_all = int(points_MST.size());
    }
    else {
        point_TSP point_in;
        points_TSP.resize(int_in);
        for (size_t i = 0; i < points_TSP.size(); ++ i) {
            std::cin >> int_in;
            point_in.x = int_in;
            std::cin >> int_in;
            point_in.y = int_in;
            points_TSP[i] = point_in;
        }
        num_all = int(points_TSP.size());
    }
    
    
}

// Effect  : check which mode is on and call it's function
void zookeeper::run() {
    if (MST_on) MST_mode();
    else if (FAST_on) FASTTSP_mode();
    else OPTTSP_mode();
}



// Effect  : Run MST mode
//           Call the MST factor and run Prim algoritm
void zookeeper::MST_mode() {
    mst_funct func(points_MST);
    prim(func, num_all);
}



// Effect  : Run FASTTSP mode
//           Call nearest_insert() function
void zookeeper::FASTTSP_mode() {
    nearest_insert();
}


// Effect  : Run OPTTSP mode
//           Procedure:
//               --> calculate the distance matrix
//               --> run FASTTSP to get a good but not best path
//               --> resize our opt_output vector
//               --> translate out TSP vector to a valid path vector
//               --> construct a valid functor for OPTTSP
//               --> run getPerms
//               --> output the resulting vector
void zookeeper::OPTTSP_mode() {
    
    // calculate matrix and run FASTTSP
    calc_matrix();
    nearest_insert();
    
    // build containers
    opt_output.resize(num_all);
    std::vector<int> path;
    int cur = 0;
    for (int i = 0; i < num_all; i ++) {
        path.push_back(cur);
        cur = TSP[cur];
    }
    std::copy(path.begin(), path.end(), opt_output.begin());
    
    // build OPTTSP valid functor and call genPerms
    opt_funct func(dis_matrix, path);
    genPerms(path, 1, func, 0);
    
    std::cout << TSP_min << '\n';
    for (int i = 0; i < num_all; i ++) {
        std::cout << opt_output[i] << " ";
    }
}



/***************************************************************************
 *                       BELOW ARE HELPER FUNCTIONS                        *
 ***************************************************************************/



// Requried: size is the number of pointer we want to do Prim with
//           comp is a valid functor comparing index
// Effect  : directly use the input vector to calculate shortest path
//           Apply Prim's Algorithm :
//                    --> https://en.wikipedia.org/wiki/Prim%27s_algorithm
//           return the shortest length of path
template <typename Functor>
double zookeeper::prim(Functor &comp, int size) {
    int count = 1;
    int cur_ind = 0;
    int min_ind = 0;
    double path_length = 0;
    double cur_min = std::numeric_limits<double>::infinity();
    std::vector<Prim_data> prim_v;
    prim_v.resize(size);
    prim_v[0].min_edge = 0;
    cur_ind = 0;
    
    
    while (count != size) {
        
        // set visited to true
        prim_v[cur_ind].visited = true;
        
        
        // For each vertex v which is unvisited
        // test whether min_edge is greater that the distance to current point
        // if it is, set min_edge to the specific distance, set prev to cur_ind
        for (int i = 0; i < size; ++i) {
            if (!prim_v[i].visited) {
                if (comp(prim_v[i].min_edge, i, cur_ind))
                    prim_v[i].prev = cur_ind;
                
                
                // From the set of vertices with is unvisited,
                // select the vertex that have the smallest tentative distantce
                if (prim_v[i].min_edge < cur_min) {
                    min_ind = i;
                    cur_min = prim_v[i].min_edge;
                }
            }
        }
        
        
        // reset min_edge to infinity
        // set current index to min index
        cur_ind = min_ind;
        cur_min = std::numeric_limits<double>::infinity();
        
        // calculate the path length
        path_length += prim_v[cur_ind].min_edge;
        
        count ++;
    }
    
    // if it's MST mode, print necassary message
    if (MST_on) {
        std::cout << path_length << '\n';
        for (int i = 1; i < size; ++ i)
            std::cout << std::min(i, prim_v[i].prev)
                      << " "
                      << std::max(i, prim_v[i].prev)
                      << '\n';
    }
    
    return path_length;
}







// Effect  : directly use the input vector to calculate shortest path
//           Apply simplified nearest_insertion's Algorithm :
//           -->
//       https://www2.isye.gatech.edu/~mgoetsch/cali/VEHICLE/TSP/TSP009__.HTM
//           print the necassary message
void zookeeper::nearest_insert() {
    int node = 3;
    int min_ind = 0;
    double distance = 0;
    double cur_min = std::numeric_limits<double>::infinity();
    TSP.resize(num_all);
    
    
    // Hinted by autograder
    // Start path with points 0, 1, 2 and it works better
    TSP[0] = 1;
    TSP[1] = 2;
    TSP[2] = 0;
    TSP_min = double_distance(0, 1) + double_distance(1, 2) +
              double_distance(2, 0);
    
    
    
    while(node < num_all) {
        
        
        // Selection step :  For convenience,
        //                   I only loop through the map to choose next
        // Insertion step :  Find the arc (i, j) in the sub-tour
        //                   which minimizes cir + crj - cij
        //                   Insert r between i and j.
        int cur_pos = 0;
        for (int i = 0; i < node; ++ i) {
            distance = double_distance(cur_pos, node) +
                       double_distance(TSP[cur_pos], node) -
                       double_distance(cur_pos, TSP[cur_pos]);
            if (distance < cur_min) {
                cur_min = distance;
                min_ind = cur_pos;
            }
            cur_pos = TSP[cur_pos];
        }
        TSP[node] = TSP[min_ind];
        TSP[min_ind] = node;
        TSP_min += cur_min;
        
        
        node ++;
        cur_min = std::numeric_limits<double>::infinity();
        
    }
    
    min_ind = 0;
    
    // print the path if it's in FASTTSP mode
    if (FAST_on) {
        std::cout << TSP_min << '\n';
        for (int i = 0; i < node; ++ i) {
            std::cout << min_ind << " ";
            min_ind = TSP[min_ind];
        }
    }
}





// Requried : ind_f and ind_s are valid index of points map
// Effect   : calculate the distance between two points
//            at index ind_f and ind_s
//            if in OPT mode --> use distance matrix
double zookeeper::double_distance(int ind_f, int ind_s) {
    if (FAST_on) {
        double x_diff = points_TSP[ind_f].x - points_TSP[ind_s].x;
        double y_diff = points_TSP[ind_f].y - points_TSP[ind_s].y;
        return sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    else return dis_matrix[ind_f][ind_s];
}



// Effect   : Calculate the ditance matrix
void zookeeper::calc_matrix() {
    double diff_x;
    double diff_y;
    double distance;
    dis_matrix.resize(num_all, std::vector<double>(num_all));
    for (int i = 0; i < num_all; i ++) {
        for (int j = 0; j < num_all; j ++) {
            diff_x = points_TSP[i].x - points_TSP[j].x;
            diff_y = points_TSP[i].y - points_TSP[j].y;
            distance = sqrt(diff_x * diff_x + diff_y * diff_y);
            dis_matrix[i][j] = distance;
            dis_matrix[j][i] = distance;
        }
    }
}


// Required : path        --> a valid resulting path from FASTTSP mode
//            permLength  --> the length that's already in perm status
//            func        --> a valid functor for OPTTSP mode
//            pathL       --> current length in perm status
// Effect   : modify vector path to optimal length path
//            variable TSP_min would be the corresponding minimal length
//            apply recursion on it's own
template <typename Functor>
void zookeeper::genPerms(std::vector<int> &path, int permLength,
                         Functor &func, double pathL) {
    if (num_all == permLength) {
        
        // connect the front and the back
        pathL += dis_matrix[path.front()][path.back()];
        // if this path the shorter, change TSP_min to this path
        // and copy this path to the output vector
        if (pathL < TSP_min) {
            TSP_min = pathL;
            std::copy(path.begin(), path.end(), opt_output.begin());
        }
        return;
    }
    if (!promising(path, permLength, func, pathL)) return;
    for (size_t i = permLength; i < path.size(); ++i) {
        std::swap(path[permLength], path[i]);
        
        //  "pathL + dis_matrix[path[permLength]][path[permLength - 1]]"
        //  is the length of our new adding node
        genPerms(path, permLength + 1, func,
                 pathL + dis_matrix[path[permLength]][path[permLength - 1]]);
        
        std::swap(path[permLength], path[i]);
    }
}


// Permutation example for understanding while implementing
/*
1 2 3

- 2 3
     3
-  3 2
     2

2 1 3
 */


// Required : path        --> a valid resulting path from FASTTSP mode
//            permLength  --> the length that's already in perm status
//            func        --> a valid functor for OPTTSP mode
//            pathL       --> current length in perm status
// Effect   : return if a premutation path is promising
//            if the length left is less or equal to 4, then promising, true
//            otherwise: use MST to calculate the lower bound of left points
//                       find two minimal path ot connect perm path to MST
//                       if the lower bound path is larger than TSP_min
//                                           --> not promising, return false
//                                           --> otherwize, return true
template <typename Functor>
bool zookeeper::promising(std::vector<int> &path, int permLength,
                          Functor &comp, double pathL) {
    if (num_all - permLength <= 4) return true;
    
    // change the variable in functor
    // let functor know which position is position 0 now
    comp.pathLength = permLength;
    
    // calculate the current lower bound length
    double sum = prim(comp, num_all - permLength) + pathL;
    double min_front = std::numeric_limits<double>::infinity();
    double min_back = std::numeric_limits<double>::infinity();
    for (int i = permLength; i < num_all; ++ i) {
        if (dis_matrix[path[path.front()]][path[i]] < min_front)
                 min_front = dis_matrix[path.front()][path[i]];
        if (dis_matrix[path[permLength - 1]][path[i]] < min_back)
                 min_back = dis_matrix[path[permLength - 1]][path[i]];
    }
    sum = sum + min_front + min_back;
    if (sum < TSP_min) return true;
    return false;
}




