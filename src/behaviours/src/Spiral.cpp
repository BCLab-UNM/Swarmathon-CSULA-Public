#include "Spiral.h"

Spiral::Spiral(){
	position.x = 0;
	position.y = 0;
	position.theta = 0;
}

Spiral::Spiral( Point position, Direction direction, bool clockwise ){
	reset(position,direction,clockwise);
}

Spiral::Spiral( Point position, Direction direction, bool clockwise, float traveldistance ){
	reset(position,direction,clockwise,traveldistance);
}

void Spiral::reset(Point position, Direction direction, bool clockwise){
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

void Spiral::reset(Point position, Direction direction, bool clockwise, float traveldistance){
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

Point Spiral::getPosition(){
	return position;
}

float Spiral::getTravelDistance(){
	return traveldistance;
}

Point Spiral::getNextPoition(){
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