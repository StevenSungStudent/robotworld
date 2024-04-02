/*
 * Lidar.cpp
 *
 *  Created on: Oct 26, 2023
 *      Author: steven
 */

#include "Lidar.hpp"

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include "MainApplication.hpp"

#include <random>

namespace Model {
	/**
	 *
	 */
	/* static */double Lidar::stddev = 10.0;
	/* static */double Lidar::lidarDegrees = 10.0;
	/**
	 *
	 */
	Lidar::Lidar(Robot &aRobot) : AbstractSensor(aRobot) {
		const Application::MainSettings &settings = Application::MainApplication::getSettings();
		Lidar::stddev = settings.getLidarError();
		Lidar::lidarDegrees = settings.getLidarDegrees();
	}
	/**
	 *
	 */
	std::shared_ptr<AbstractStimulus> Lidar::getStimulus() const {
		DistanceStimuli distanceStimuli;
		Robot *robot = dynamic_cast<Robot*>(agent);
		if (robot) {
			std::random_device rd { };
			std::mt19937 gen { rd() };
			std::normal_distribution<> noise { 0, Lidar::stddev };

			std::vector<WallPtr> walls = RobotWorld::getRobotWorld().getWalls();
			for (double i = 0; i < 360; i += Lidar::lidarDegrees) {
				double distance = noDistance;
				double angle = Utils::MathUtils::toRadians(i);

				for (std::shared_ptr<Wall> wall : walls) {
					wxPoint wallPoint1 = wall->getPoint1();
					wxPoint wallPoint2 = wall->getPoint2();
					wxPoint robotLocation = robot->getPosition();

					wxPoint laserEndpoint { static_cast<int>(robotLocation.x + std::cos(angle) * lidarBeamLength), static_cast<int>(robotLocation.y + std::sin(angle) * lidarBeamLength) };

					wxPoint interSection = Utils::Shape2DUtils::getIntersection(wallPoint1, wallPoint2, robotLocation, laserEndpoint);

					if (interSection != wxDefaultPosition) {
						double temp = Utils::Shape2DUtils::distance(robotLocation, interSection);
						if (temp < distance || distance == noDistance) {
							distance = temp;
						}
					}
				}
				if (distance == noDistance) {
					distanceStimuli.stimuli.push_back(DistanceStimulus(angle, noDistance));
				} else {
					distanceStimuli.stimuli.push_back(DistanceStimulus(angle, distance + noise(gen)));
				}
			}
			return (std::make_shared<DistanceStimuli>(distanceStimuli));
		}
		return (std::make_shared<DistanceStimulus>(noAngle, noDistance));
	}
	/**
	 *
	 */
	std::shared_ptr<AbstractPercept> Lidar::getPerceptFor(std::shared_ptr<
			AbstractStimulus> anAbstractStimulus) const {
		Robot *robot = dynamic_cast<Robot*>(agent);
		if (robot) {

			DistanceStimuli *distanceStimuli = dynamic_cast<DistanceStimuli*>(anAbstractStimulus.get());
			if (distanceStimuli) {
				DistancePercepts distancePercepts;

				for (DistanceStimulus distanceStimulus : distanceStimuli->stimuli) {
					if (distanceStimulus.distance != noDistance) {
						wxPoint endpoint { static_cast<int>(std::cos(distanceStimulus.angle) * distanceStimulus.distance), static_cast<int>(std::sin(distanceStimulus.angle) * distanceStimulus.distance) };
						distancePercepts.pointCloud.push_back(DistancePercept(endpoint));
					} else {
						distancePercepts.pointCloud.push_back(DistancePercept(wxPoint(noDistance, noDistance)));
					}
				}
				return (std::make_shared<DistancePercepts>(distancePercepts));
			}
		}

		return std::make_shared<DistancePercept>(wxPoint(invalidDistance, invalidDistance));
	}
	/**
	 *
	 */
	std::string Lidar::asString() const {
		return "Lidar";
	}
	/**
	 *
	 */
	std::string Lidar::asDebugString() const {
		return asString();
	}
} // namespace Model
