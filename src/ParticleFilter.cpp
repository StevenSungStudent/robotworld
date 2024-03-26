/*
 * ParticleFilter.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#include "ParticleFilter.hpp"
#include "Robot.hpp"
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
		particleCloud.push_back(Particle(wxPoint(value(gen), value(gen)), 0));
	}
}

void Model::ParticleFilter::updateParticles() {
	std::vector<WallPtr> walls = RobotWorld::getRobotWorld().getWalls();
	for (unsigned long i = 0; i < particleCloud.size(); ++i) {
		PointCloud tempPointCloud;
		wxPoint currentPosition = particleCloud.at(i).position;

		for (unsigned short j = 0; j < 360; j += 2) {
			double angle = Utils::MathUtils::toRadians(j);
			wxPoint intersect = wxPoint(noDistance, noDistance);

			for (std::shared_ptr<Wall> &wall : walls) {

				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				wxPoint laserEndpoint { static_cast<int>(currentPosition.x + std::cos(angle) * lidarBeamLength), static_cast<int>(currentPosition.y + std::sin(angle) * lidarBeamLength) };
				wxPoint newInterSection = Utils::Shape2DUtils::getIntersection(wallPoint1, wallPoint2, currentPosition, laserEndpoint);

				if (newInterSection != wxDefaultPosition) {
					double newIntersectDistance = Utils::Shape2DUtils::distance(currentPosition, newInterSection);

					if (Utils::Shape2DUtils::distance(currentPosition, intersect) < newIntersectDistance || intersect.x == noDistance) {
						intersect = newInterSection;
					}
				}
			}
			tempPointCloud.push_back(DistancePercept(intersect));
		}
		particleCloud.at(i).pointCloud = tempPointCloud;
	}

	moveParticles();
	resample();
}

void Model::ParticleFilter::moveParticles() {
	Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot("Robot");
	if (robot) {
		std::random_device randomDevice;
		std::mt19937 gen(randomDevice());
		wxPoint currentPosition = robot->getPosition();
		wxPoint previousPosition = robot->getPreviousPosition();
		wxPoint difference = currentPosition - previousPosition;

		for (unsigned long i = 0; i < particleCloud.size(); ++i) {
			std::normal_distribution<double> distrobution(0.0, 10.0);
			wxPoint noise = wxPoint(distrobution(gen), distrobution(gen));
			particleCloud.at(i).position += difference + noise;
//			std::cout << particleCloud.at(i).position << std::endl;
		}
	}

}

void Model::ParticleFilter::resample() {
	Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot("Robot");
	if (robot) {
		double totalWeight = 0;
		std::random_device randomDevice;
		std::mt19937 gen(randomDevice());
		std::vector<double> weightList;
		std::vector<Particle> resample;

		PointCloud robotPointCloud = robot->currentLidarPointCloud;

		for (Particle &particle : particleCloud) {
			double difference = 0;

			for (unsigned long i = 0; i < robotPointCloud.size(); ++i) {
				if(!std::abs(Utils::Shape2DUtils::distance(robotPointCloud.at(i).point, particle.pointCloud.at(i).point)) == 0){
					difference += std::abs(Utils::Shape2DUtils::distance(robotPointCloud.at(i).point, particle.pointCloud.at(i).point));
				}

			}
			particle.weight = 1 / (1 + difference);
			totalWeight += particle.weight;
		}

		for (Particle &particle : particleCloud) {//TODO: normalizing the weights is not needed i think.
			particle.weight = particle.weight / totalWeight;
			std::cout << particle.weight << std::endl;
			weightList.push_back(particle.weight);

		}
		std::discrete_distribution<int> distrobution(weightList.begin(), weightList.end());

		for(unsigned long i = 0; i < particleCloud.size(); ++i){
			resample.push_back(particleCloud.at(distrobution(gen)));
		}
		particleCloud = resample;
	}
}
