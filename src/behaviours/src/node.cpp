#include "node.h"
#include <math.h>
using namespace std;

node::node(int xp, int yp, float d, float p) 
{xPos=xp; yPos=yp; level=d; priority=p;}
    
int node::getxPos() const {return xPos;}
int node::getyPos() const {return yPos;}
float node::getLevel() const {return level;}
float node::getPriority() const {return priority;}

void node::addcostToPriority(float cost)
{
    priority+=cost;
}

void node::updatePriority(const int & xDest, const int & yDest)
{
    priority=level+estimate(xDest, yDest)*10; //A*
}

// give better priority to going strait instead of diagonally
void node::nextLevel(const float & i) // i: direction
{   
    //commented out is what gave better priority to going strait instead of diagonally
    //const int dir=8; // number of possible directions to go at any position
    // level+=(dir==8?(i%2==0?10:14):10);

    //
    level+=0;
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



// // Determine priority (in the priority queue)
// bool node::operator<(const node & b)
// {
//     return priority > b.getPriority();
// }

// bool node::operator> (const ){
//     return  priority < b.getPriority();
// }

// bool node::operator== (const node & b){
//     return priority == b.getPriority();