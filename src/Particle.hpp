/*
 * Particle.hpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#ifndef SRC_PARTICLE_HPP_
#define SRC_PARTICLE_HPP_

#include "Shape2DUtils.hpp"


class Particle {
public:
	Particle();
	virtual ~Particle();

	wxPoint position;
	double weight;

	/**
	 * Returns a 1-line description of the object
	 */
	virtual std::string asString() const;
};

#endif /* SRC_PARTICLE_HPP_ */
