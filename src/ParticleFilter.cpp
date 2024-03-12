/*
 * ParticleFilter.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#include "ParticleFilter.hpp"

Model::ParticleFilter::ParticleFilter() {
	// TODO Auto-generated constructor stub

}

Model::ParticleFilter::~ParticleFilter() {
	// TODO Auto-generated destructor stub
}

void Model::ParticleFilter::generateParticles(const unsigned long& amount){
	std::random_device rd{};
	std::mt19937 gen{rd()};
    std::normal_distribution<> value{0, 512};

    for(unsigned long i = 0; i < amount; ++i){
    	particleCloud.push_back(DistancePercept(wxPoint(value(gen), value(gen))));
    }
}
