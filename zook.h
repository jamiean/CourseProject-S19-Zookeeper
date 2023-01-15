//  EECS-281 Spring 2018 Project 4
//  zook.h
//
//  JunCheng An; jamiean
//
//  Created by JunCheng An on 6/8/18.
//  Copyright © 2018 JunCheng An. All rights reserved.
//

#ifndef zook_h
#define zook_h

#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include "math.h"


class Invalidcage{};
class ModeUnfound{};
class HelpFunctionCalled{};


class zookeeper {
public:
    
    // Required: argv and argv are valid command line information
    // Effect  : read command line input, see what mode is on
    //           -h    : helper function called, throw HelpFunctionCalled{};
    //           -m XX : calculate mode
    //                   XX -> MST: turn MST_on to TRUE
    //                   XX -> OPTTSP : turn OPT_on to TRUE
    //                   XX -> FASTTSP: turn FAST_on to TRUE
    //                   XX -> otherwise, throw ModeUnfound{}
    void read_command(int argc, char* argv[]);
    
    
    // Modifier: cin
    // Effect  : read in map information from standard input
    //           if there're cages in both safe and wild area
    //              but no cage in intersection area in MST mode
    //           ---> throw Invalidcage{};
    //           MST       --> read in points_MST
    //           FAST/OPT  --> read in points_TSP
    void read_map();
    
    
    // Effect  : check which mode is on and call it's function
    void run();
    
    
    

private:
    
    
    // General map data structure for MST
    // x    -> x-coordinate of the point
    // y    -> y-coordinate of the point
    // type ->   i : intersection of wild and safe area
    //           w : wild area
    //           s : safe area
    struct point_MST {
        int x = 0;
        int y = 0;
        char type;
    };
    
    
    // General map data structure for TSP
    // x    -> x-coordinate of the point
    // y    -> y-coordinate of the point
    struct point_TSP {
        int x = 0;
        int y = 0;
    };
    
    
    // General data structure for running prim algorithm
    // visited  ->  if the points have been visited
    // min_edge ->  current min distance to the point
    // prev     ->  it's min distance previous point
    struct Prim_data {
        bool visited = false;
        double min_edge = std::numeric_limits<double>::infinity();
        int prev = -1;
    };
    
    
    
    // Containers
    std::vector<int> TSP;
    std::vector<point_MST> points_MST;
    std::vector<point_TSP> points_TSP;
    std::vector<std::vector<double>> dis_matrix;
    std::vector<int> opt_output;
    
    // Number of points
    int num_all;
    
    // current MIN TSP path length
    double TSP_min = 0;
    
    // Mode Indicator
    bool MST_on = false;
    bool OPT_on = false;
    bool FAST_on = false;
    
    
    
    
    // Effect  : Run MST mode
    //           Call the MST factor and run Prim algoritm
    void MST_mode();
    
    
    
    // Effect  : Run FASTTSP mode
    //           Call nearest_insert() function
    void FASTTSP_mode();
    
    
    // Effect  : Run OPTTSP mode
    //           Procedure:
    //               --> calculate the distance matrix
    //               --> run FASTTSP to get a good but not best path
    //               --> resize our opt_output vector
    //               --> translate out TSP vector to a valid path vector
    //               --> construct a valid functor for OPTTSP
    //               --> run getPerms
    //               --> output the resulting vector
    void OPTTSP_mode();
    
    
    
    
/***************************************************************************
 *                       BELOW ARE HELPER FUNCTIONS                        *
***************************************************************************/
    
    
    
    // Functor for MST comparasion
    // Overload () operator, it has four parameter
    //          --> (min, cur, com)
    //          -->  min: current min distance
    //          -->  cur: current point index
    //          -->  com: new comparing point index
    class mst_funct {
    public:
        mst_funct(std::vector<point_MST> &v_in): points_ref(v_in) {}
        
        bool operator()(double &min, int cur, int com) {
            if ((points_ref[cur].type == 'w' && points_ref[com].type == 's') ||
                (points_ref[cur].type == 's' && points_ref[com].type == 'w'))
                return false;
            x_diff_new = abs(points_ref[cur].x - points_ref[com].x);
            y_diff_new = abs(points_ref[cur].y - points_ref[com].y);
            dis_new = sqrt(x_diff_new * x_diff_new + y_diff_new * y_diff_new);
            if (dis_new < min) {
                min = dis_new;
                return true;
            }
            return false;
        }
        
        
    private:
        const std::vector<point_MST> &points_ref;
        double x_diff_new = 0;
        double y_diff_new = 0;
        double dis_new = 0;
    
    };
    
    
    // Functor for OPTTSP comparasion
    // Overload () operator, it has four parameter
    //          --> (min, cur, com)
    //          -->  min: current min distance
    //          -->  cur: current point index
    //          -->  com: new comparing point index
    // path_ref is the reference of original FASTTSP path we have
    // dis_ref  is the reference of our distance matrix
    class opt_funct {
    public:
        opt_funct(std::vector<std::vector<double>> &v_in,
                  std::vector<int> &path_in): dis_ref(v_in), path_ref(path_in) {}
        
        bool operator()(double &min, int cur, int com) {
            cur += pathLength;
            com += pathLength;
            if (min < dis_ref[path_ref[cur]][path_ref[com]]) return false;
            else {
                min = dis_ref[path_ref[cur]][path_ref[com]];
                return true;
            }
        }
        
        int pathLength = 0;
        
    private:
        const std::vector<std::vector<double>> &dis_ref;
        const std::vector<int> &path_ref;
        
    };
    
    
    
    
    
    // Requried: size is the number of pointer we want to do Prim with
    //           comp is a valid functor comparing index
    // Effect  : directly use the input vector to calculate shortest path
    //           Apply Prim's Algorithm :
    //                    --> https://en.wikipedia.org/wiki/Prim%27s_algorithm
    //           return the shortest length of path
    template <typename Functor>
    double prim(Functor &comp, int size);
    
    
    
    
    // Effect  : directly use the input vector to calculate shortest path
    //           Apply simplified nearest_insertion's Algorithm :
    //           -->
    //       https://www2.isye.gatech.edu/~mgoetsch/cali/VEHICLE/TSP/TSP009__.HTM
    //           print the necassary message
    void nearest_insert();
    
    

    
    
    // Requried : ind_f and ind_s are valid index of points map
    // Effect   : calculate the distance between two points
    //            at index ind_f and ind_s
    //            if in OPT mode --> use distance matrix
    double double_distance(int ind_f, int ind_s);
    
    
    
    // Effect   : Calculate the ditance matrix
    void calc_matrix();
    
    
    
    // Required : path        --> a valid resulting path from FASTTSP mode
    //            permLength  --> the length that's already in perm status
    //            func        --> a valid functor for OPTTSP mode
    //            pathL       --> current length in perm status
    // Effect   : modify vector path to optimal length path
    //            variable TSP_min would be the corresponding minimal length
    //            apply recursion on it's own
    template <typename Functor>
    void genPerms(std::vector<int> &path, int permLength,
                  Functor &func, double pathL);
    
    
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
    bool promising(std::vector<int> &path, int permLength,
                   Functor &comp, double pathL);
    
    
};






#endif /* zook_h */
