/*
 * Particle.hpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#ifndef SRC_PARTICLE_HPP_
#define SRC_PARTICLE_HPP_

#include "DistancePercepts.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include <vector>

namespace Model {
	class Particle {
	public:
		Particle(const wxPoint &aPostion, const double &aWeight);
		virtual ~Particle();

		wxPoint position;
		double weight;

		PointCloud pointCloud;

		void generatePointCloud();

		/**
		 * Returns a 1-line description of the object
		 */
		virtual std::string asString() const;

	private:
		static double lidarDegrees;
	};
}

#endif /* SRC_PARTICLE_HPP_ */
