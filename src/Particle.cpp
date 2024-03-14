/*
 * Particle.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#include "Particle.hpp"
#include <sstream>

Particle::Particle(const wxPoint &aPosition, const double &aWeight) : position(aPosition), weight(aWeight) {
	// TODO Auto-generated constructor stub

}

Particle::~Particle() {
	// TODO Auto-generated destructor stub
}

/**
 *
 */
std::string Particle::asString() const
{
	std::stringstream result;
	result << "position x: " << position.x  << " position y: " << position.y << " weight: " << weight;
	return result.str();
}
