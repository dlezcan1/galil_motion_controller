//
//  main.cpp
//  GalilMotionController
//
//  Created by Dimitri Lezcano on 2/23/22.
//

#include <iostream>
#include "GalilController.h"

void printCommandResponse(GalilController* controller, GCStringIn command)
{
    std::cout << "Sending command:   \"" << command << "\"\n";
    std::cout << "Response received: \"" << controller->command(command) << "\"\n\n";
}

void printCommandResponse(GalilController* controller, std::string command)
{
    printCommandResponse(controller, command.c_str());
}

int main(int argc, const char * argv[]) {
    
    GalilController* gc = new GalilController("192.168.1.201");
    
    // functioning get position
    std::cout << "Get the positions of the axes.\n";
    printCommandResponse(gc, "PR ,?,,?,?");
    bool axes[5] = {false, true, false, true, true};
    long* positions = gc->getPosition(axes, false);
    for (int i = 0; i < 5; i++)
        std::cout << "Axes position [" << i << "] = " << positions[i] << std::endl;
    
    // String input and output
//    std::string command;
//    while (true)
//    {
//        std::cout << "Galil Controller Message: ";
//        std::getline(std::cin, command);
//        // std::cout << command << std::endl;
//        printCommandResponse(gc, command);
//        
//    } // while
    
    delete gc;

    return 0;
}
