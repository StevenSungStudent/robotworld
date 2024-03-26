/*
 * Particle.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#include "RobotWorld.hpp"
#include "Particle.hpp"
#include "Robot.hpp"
#include <sstream>

namespace Model {
	Particle::Particle(const wxPoint &aPosition, const double &aWeight) :
			position(aPosition), weight(aWeight) {
		// TODO Auto-generated constructor stub

	}

	Particle::~Particle() {
		// TODO Auto-generated destructor stub
	}

	void Particle::generatePointCloud() {
		const short int lidarBeamLength = 1024;

		std::vector<WallPtr> walls = RobotWorld::getRobotWorld().getWalls();
		PointCloud tempPointCloud;

		for (unsigned short j = 0; j < 360; j += 2) {
			double angle = Utils::MathUtils::toRadians(j);
			double distance = noDistance;

			for (std::shared_ptr<Wall> &wall : walls) {

				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				wxPoint laserEndpoint { static_cast<int>(position.x + std::cos(angle) * lidarBeamLength), static_cast<int>(position.y + std::sin(angle) * lidarBeamLength) };
				wxPoint intersect = Utils::Shape2DUtils::getIntersection(wallPoint1, wallPoint2, position, laserEndpoint);

				if (intersect != wxDefaultPosition) {
					double newDistance = Utils::Shape2DUtils::distance(position, intersect);

					if (distance < newDistance || distance == noDistance) {
						distance = newDistance;
					}
				}
			}
			if(distance == noDistance){
				tempPointCloud.push_back(DistancePercept(wxPoint(noObject,noObject)));
			}else{
				wxPoint endpoint(static_cast< int >( std::cos( angle) * distance), static_cast< int >( std::sin( angle) * distance));
				tempPointCloud.push_back(DistancePercept(endpoint));
			}
		}
		pointCloud = tempPointCloud;
	}

	/**
	 *
	 */
	std::string Particle::asString() const {
		std::stringstream result;
		result << "position x: " << position.x << " position y: " << position.y << " weight: " << weight;
		return result.str();
	}
}
