/*
 * ParticleFilter.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#include "ParticleFilter.hpp"

#include <iostream>

Model::ParticleFilter::ParticleFilter() {
	// TODO Auto-generated constructor stub

}

Model::ParticleFilter::~ParticleFilter() {
	// TODO Auto-generated destructor stub
}

void Model::ParticleFilter::generateParticles(const unsigned long &amount) {
	std::random_device rd { };
	std::mt19937 gen { rd() };
	std::uniform_int_distribution<> value { 0, 1024 };

	for (unsigned long i = 0; i < amount; ++i) {
		particleCloud.push_back(Particle(wxPoint(value(gen), value(gen)), 1));
	}
}

void Model::ParticleFilter::updateParticles() {
	for (unsigned long i = 0; i < particleCloud.size(); ++i) {
		particleCloud.at(i).pointCloud.clear();
		wxPoint currentPosition = particleCloud.at(i).position;
		std::vector<WallPtr> walls = RobotWorld::getRobotWorld().getWalls();

		for (unsigned short j = 0; j < 360; j += 2) {
			double angle = Utils::MathUtils::toRadians(j);

			for (std::shared_ptr<Wall> wall : walls) {
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();

				wxPoint laserEndpoint { static_cast<int>(currentPosition.x + std::cos(angle) * lidarBeamLength), static_cast<int>(currentPosition.y + std::sin(angle) * lidarBeamLength) };

				wxPoint interSection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, currentPosition, laserEndpoint);
				if (interSection != wxDefaultPosition) {
//					std::cout << "current point: "<< currentPosition << " intersect: " << interSection << std::endl;
					particleCloud.at(i).pointCloud.push_back(DistancePercept(interSection));
				}
			}
		}
	}
}
