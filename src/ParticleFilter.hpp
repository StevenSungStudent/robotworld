/*
 * ParticleFilter.hpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#ifndef SRC_PARTICLEFILTER_HPP_
#define SRC_PARTICLEFILTER_HPP_

#include "Particle.hpp"
#include <vector>
#include <random>

namespace Model{

	class ParticleFilter {
	public:
		explicit ParticleFilter();
		virtual ~ParticleFilter();

		void generateParticles(const unsigned long& amount);

	private:
		std::vector<Particle> pointcloud;

	};
}

#endif /* SRC_PARTICLEFILTER_HPP_ */
