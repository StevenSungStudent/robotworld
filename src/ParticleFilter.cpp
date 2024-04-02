/*
 * ParticleFilter.cpp
 *
 *  Created on: Oct 27, 2023
 *      Author: steven
 */

#include "ParticleFilter.hpp"
#include "Robot.hpp"
#include <iostream>

namespace Model {
	ParticleFilter::ParticleFilter() {
	}

	ParticleFilter::~ParticleFilter() {
	}

	void ParticleFilter::generateParticles(const unsigned long &amount) {
		std::random_device rd { };
		std::mt19937 gen { rd() };
		std::uniform_int_distribution<> value { 0, 1024 };

		for (unsigned long i = 0; i < amount; ++i) {
			particleCloud.push_back(Particle(wxPoint(value(gen), value(gen)), 0));
		}
	}

	void ParticleFilter::updateParticles() {
		for (unsigned long i = 0; i < particleCloud.size(); ++i) {
			particleCloud.at(i).generatePointCloud();
		}

		moveParticles();
		resample();
	}

	void ParticleFilter::moveParticles() {
		static wxPoint lastMeasurement = wxPoint(noDistance,noDistance);
		RobotPtr robot = RobotWorld::getRobotWorld().getRobot("Robot");
		if (robot) {
			std::random_device randomDevice;
			std::mt19937 gen(randomDevice());
			wxPoint currentPosition = robot->getPosition();
			wxPoint previousPosition = robot->getPreviousPosition();

			if(lastMeasurement.x == noDistance || lastMeasurement != currentPosition){
				lastMeasurement = currentPosition;
				wxPoint difference = currentPosition - previousPosition;

				for (unsigned long i = 0; i < particleCloud.size(); ++i) {
					std::normal_distribution<double> distrobution(0.0, 3.0);
					wxPoint noise = wxPoint(static_cast<int>(distrobution(gen)), static_cast<int>(distrobution(gen)));
					particleCloud.at(i).position += difference + noise;
				}
			}

		}

	}

	void ParticleFilter::resample() {
		RobotPtr robot = RobotWorld::getRobotWorld().getRobot("Robot");
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
					if (!std::abs(Utils::Shape2DUtils::distance(robotPointCloud.at(i).point, particle.pointCloud.at(i).point)) == 0) {
						difference += std::abs(Utils::Shape2DUtils::distance(robotPointCloud.at(i).point, wxPoint(0, 0)) - Utils::Shape2DUtils::distance(particle.pointCloud.at(i).point, wxPoint(0,0)));
					}
				}
				particle.weight = 1 / (1 + difference);
				totalWeight += particle.weight;
			}

			for (Particle &particle : particleCloud) {
				weightList.push_back(particle.weight / totalWeight);
			}

			std::discrete_distribution<int> distrobution(weightList.begin(), weightList.end());
			for (unsigned long i = 0; i < particleCloud.size(); ++i) {
				resample.push_back(particleCloud.at(distrobution(gen)));
			}
			particleCloud = resample;
		}
	}
}
