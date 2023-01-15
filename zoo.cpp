//  EECS-281 Spring 2018 Project 4
//  zoo.cpp
//
//  JunCheng An; jamiean
//
//  Created by JunCheng An on 6/8/18.
//  Copyright © 2018 JunCheng An. All rights reserved.
//

#include <iomanip>
#include "zook.h"


int main(int argc, char* argv[]) {
    
    
    // for running with xcode
    #ifdef __APPLE__
    if(!freopen("sample-ab.txt", "r", stdin)) {
        std::cerr << "THERE IS NO INPUT!" << std::endl;
        exit(1);
    }
    //freopen("myoutput.txt", "w", stdout);
    #endif
    
     
    std::ios_base::sync_with_stdio(false);
    
    // Always show 2 decimal places
    std::cout << std::setprecision(2);
    
    // Disable scientific notation for large numbers
    std::cout << std::fixed;
    
    
    
    try {

        zookeeper zoo;
        
        zoo.read_command(argc, argv);
        
        zoo.read_map();

        zoo.run();
        
        
    }
    catch(HelpFunctionCalled &e) {
        std::cerr << "Helper function called!" << std::endl;
        exit(1);
    }
    catch(ModeUnfound &h) {
        std::cerr << "Wrong Mode Input!" << std::endl;
        exit(1);
    }
    catch(Invalidcage &i) {
        std::cerr << "Cage error!" << std::endl;
        exit(1);
    }
    
    return 0;
}
