/*
 * ParticleFilter.hpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include "Particle.hpp"
#include "DistancePercepts.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"

#include <vector>
#include <random>

namespace Model{

	class ParticleFilter {
	public:
		explicit ParticleFilter();
		virtual ~ParticleFilter();

		void generateParticles(const unsigned long& amount);

		void updateParticles();

		const std::vector<Particle>& getParticleCloud() const {
			return particleCloud;
		}

	private:
		std::vector<Particle> particleCloud;
		const short int lidarBeamLength = 1024;

		void moveParticles();
		void calculateWeight();
		void resample();

	};
}

#endif /* SRC_PARTICLEFILTER_HPP_ */
