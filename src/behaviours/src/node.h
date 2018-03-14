#ifndef NODE_H
#define NODE_H
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <time.h>
#include <sstream>
#include <vector>
class node
{


    public:
        // current position
        int xPos;
        int yPos;
        // total distance already travelled to reach the node
        float level;
        // priority=level+remaining distance estimate
        float priority;  // smaller: higher priority
        node(int xp, int yp, float d, float p);
    
        int getxPos() const;
        int getyPos() const;
        float getLevel() const;
        float getPriority() const;

        void addcostToPriority(float cost);

        void updatePriority(const int & xDest, const int & yDest);

        // give better priority to going strait instead of diagonally
        void nextLevel(const float & i); // i: direction
        
        // Estimation function for the remaining distance to the goal.
        float & estimate (const int & xDest, const int & yDest) ;

        // bool operator<(const node & b);
};

#endif // NODE_H