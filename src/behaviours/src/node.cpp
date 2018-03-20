#include "node.h"
#include <math.h>
using namespace std;

node::node(int xp, int yp, float d, float p, int pd) 
{xPos=xp; yPos=yp; level=d; priority=p; parentDirection=pd;}
    
int node::getxPos() const {return xPos;}
int node::getyPos() const {return yPos;}
float node::getLevel() const {return level;}
float node::getPriority() const {return priority;}

float node::getParentDirection() const {return parentDirection;}

void node::addcostToPriority(float cost)
{
    priority+=cost;
}

void node::updatePriority(const int & xDest, const int & yDest)
{
    priority=level+estimate(xDest, yDest)*10; //A*
}


// give better priority to going in the same direction instead of changing direction.
// This is done since the fewer waypoints and turns will lead to less errors
void node::nextLevel(const float & i) // i: direction
{   
    //commented out is what gave better priority to going straight instead of diagonally
    // const float dir=8; // number of possible directions to go at any position
    // level+=(dir==8?(i%2==0?10:14):10);


    if(i==parentDirection){
        level-=10.0;
    }
    else if(parentDirection==-1){level-=10.0;}
    else{level+=0;}
}
        
// Estimation function for the remaining distance to the goal.
float & node::estimate(const int & xDest, const int & yDest) 
{
    static int xd, yd;
    static float d;
    xd=xDest-xPos;
    yd=yDest-yPos;         

    // Euclidian Distance
    d=static_cast<float>(sqrt(xd*xd+yd*yd));

    // Manhattan distance
    //d=abs(xd)+abs(yd);
            
    // Chebyshev distance
            //d=max(abs(xd), abs(yd));

    return(d);
}

