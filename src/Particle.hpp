/*
 * Particle.hpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#ifndef SRC_PARTICLE_HPP_
#define SRC_PARTICLE_HPP_

#include "Shape2DUtils.hpp"
#include "DistancePercepts.hpp"


class Particle {
public:
	Particle(const wxPoint &aPostion, const double &aWeight);
	virtual ~Particle();

	wxPoint position;
	double weight;

	Model::PointCloud pointCloud;

	/**
	 * Returns a 1-line description of the object
	 */
	virtual std::string asString() const;
};

#endif /* SRC_PARTICLE_HPP_ */
