#include <iostream>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <random>
#include "Point.h"

#include <random_numbers/random_numbers.h>


class RandomSelector {
    int accuracy = 2;
    float areasize = 3.5;
    float areamargin = 0.5;
    random_numbers::RandomNumberGenerator* rng;
  public:

    float getAreaSize(){
        return areasize;
    }


    int getAccuracy(){
        return accuracy;
    }


    float getAreaMargin(){
        return areamargin;
    }

    void setAccuracy(int acc){
        accuracy = acc;
    }

    void setAreaSize(float size){
        areasize = size;
    }

    void setAreaMargin(float margin){
        areamargin = margin;
    }

    RandomSelector() { 
//        rng = new random_numbers::RandomNumberGenerator(0); 
        rng = new random_numbers::RandomNumberGenerator(); 
    }

    RandomSelector(float acc, float size, float margin){
        accuracy = acc;
        areasize = size;
        areamargin = margin;
//        rng = new random_numbers::RandomNumberGenerator(0); 
        rng = new random_numbers::RandomNumberGenerator(); 
    }

    Point getZone(int index){
        Point position;
        position.x = areasize;
        position.y = areasize;

        int counter = 1;
        int maxcount = 1;
        int count = 0;
        bool turn = true;

        int distance = areasize;
        Direction direction = West;
        
        //std::cout << "\nx: " << position.x << "\ty: " << position.y;

        for (int i = 0; i < index; i++){

            if(counter % 2 == 0 && counter > 0) {
                maxcount++;
                counter = 0;
            }
            switch(direction)
            {
                case North:
                    position.y += distance;
                    if(turn){
                        direction = West;
                        turn = false;
                    }
                    break;
                case East:
                    position.x += distance;
                    if(turn){
                        direction = North;
                        turn = false;
                    }
                    break;
                case South:
                    position.y -= distance;
                    if(turn){
                        direction = East;
                        turn = false;
                    }
                    break;
                case West:
                    position.x -= distance;
                    if(turn){
                        direction = South;
                        turn = false;
                    }
                    break;
                default:
                    break;
            }

            count++;
            if(maxcount == count) {
                count = 0;
                counter++;
                turn = true;
            }

            //cout << "\nx: " << position.x << "\ty: " << position.y;
            //cout << "\ndirection " << direction;

        }
        return position;
    }

    Point getRandomPoint(Point zone){
        float min_x = zone.x - areasize + areamargin;
        float max_x = zone.x - areamargin;
        
        float min_y = zone.y - areasize + areamargin;
        float max_y = zone.y - areamargin;
    
        Point p;        
        p.x = RandomNumber(min_x, max_x);
        p.y = RandomNumber(min_y, max_y);
        return p;
    }

    Point getRandomPointInZone(int zone){
        return getRandomPoint(getZone(zone));
    }

    float RandomNumber(float Min, float Max){
        float f =  float(rng->uniformReal( Min, Max ));
        f *= pow(10, accuracy);
        f = ceil(f);
        f /= pow(10, accuracy);
        return f;
    }
};
