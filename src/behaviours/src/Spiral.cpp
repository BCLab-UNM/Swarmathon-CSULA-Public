#include <iostream>
#include <angles/angles.h>

using namespace std;

enum Direction { North, East, South, West };

class Spiral
{
	private: 
		// Position in 3D coords
		Point position;
		bool clockwise = true;
		Direction direction = North;
		int counter = 0;
		float distance = 0;
		float traveldistance = 1;

	public:

	Spiral(){
		position.x = 0;
		position.y = 0;
		position.theta = 0;
	}

	Spiral( Point position, Direction direction, bool clockwise ){
		reset(position,direction,clockwise);
	}

	Spiral( Point position, Direction direction, bool clockwise, float traveldistance ){
		reset(position,direction,clockwise,traveldistance);
	}

	void reset(Point position, Direction direction, bool clockwise){
		this->position = position;
		this->direction = direction;
		this->clockwise = clockwise;
		this->counter = 0;
		this->distance = 0;
		this->traveldistance = 1;
		switch(direction)
		{
			case North:
				position.theta = M_PI/2;
				break;
			case East:
				position.theta = 0;
				break;
			case South:
				position.theta = 2 * M_PI/3;
				break;
			case West:
				position.theta = M_PI;
				break;
			default:
				break;
		}
	}

	void reset(Point position, Direction direction, bool clockwise, float traveldistance){
		this->position = position;
		this->direction = direction;
		this->clockwise = clockwise;
		this->counter = 0;
		this->distance = 0;
		this->traveldistance = traveldistance;
		switch(direction)
		{
			case North:
				position.theta = M_PI/2;
				break;
			case East:
				position.theta = 0;
				break;
			case South:
				position.theta = 2 * M_PI/3;
				break;
			case West:
				position.theta = M_PI;
				break;
			default:
				break;
		}
	}

	Point getPosition(){
		return position;
	}

	float getTravelDistance(){
		return traveldistance;
	}

	Point getNextPoition(){
		if(counter % 2 == 0) {
			distance += traveldistance;
		}
		switch(direction)
		{
			case North:
				position.y += distance;
				if (clockwise){
					direction = East;
				}
				else{
					direction = West;
				}
				break;
			case East:
				position.x += distance;
				if (clockwise){
					direction = South;
				}
				else{
					direction = North;
				}
				break;
			case South:
				position.y -= distance;
				if (clockwise){
					direction = West;
				}
				else{
					direction = East;
				}
				break;
			case West:
				position.x -= distance;
				if (clockwise){
					direction = North;
				}
				else{
					direction = South;
				}
				break;
			default:
				break;

		}
		switch(direction)
		{
			case North:
				position.theta = M_PI/2;
				break;
			case East:
				position.theta = 0;
				break;
			case South:
				position.theta = 2 * M_PI/3;
				break;
			case West:
				position.theta = M_PI;
				break;
			default:
				break;
		}
		counter ++;
		return position;
	}

};
