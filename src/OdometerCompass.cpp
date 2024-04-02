/*
 * OdometerCompass.cpp
 *
 *  Created on: Oct 4, 2023
 *      Author: steven
 */

#include "OdometerCompass.hpp"
#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include "MainApplication.hpp"

namespace Model {
	/**
	 *
	 */
	/* static */double OdometerCompass::stddevCompass = 0.0349066; // 2 degrees
	/* static */double OdometerCompass::stddevOdometer = 1;
	/**
	 *
	 */
	OdometerCompass::OdometerCompass(Robot &aRobot) : AbstractSensor(aRobot) {
		Application::MainSettings& settings = Application::MainApplication::getSettings();
		OdometerCompass::stddevCompass = settings.getCompassError();
		OdometerCompass::stddevOdometer = settings.getOdometerError();
	}
	/**
	 *
	 */
	std::shared_ptr<AbstractStimulus> OdometerCompass::getStimulus() const {
		Robot *robot = dynamic_cast<Robot*>(agent);
		if (robot) {
			std::random_device rd { };
			std::mt19937 gen { rd() };
			std::normal_distribution<> compassError { 0, OdometerCompass::stddevCompass };
			std::normal_distribution<> odometerError { 0, OdometerCompass::stddevOdometer };

			double angle = Utils::Shape2DUtils::getAngle(robot->getFront()) + compassError(gen);

			wxPoint robotLocation = robot->getPosition();
			wxPoint robotPreviousLocation = robot->getPreviousPosition();
			double odometerNoise = odometerError(gen);

			double distance = Utils::Shape2DUtils::distance(robotLocation, robotPreviousLocation) + odometerNoise;

			return (std::make_shared<DistanceStimulus>(angle, distance));
		}
		return (std::make_shared<DistanceStimulus>(noAngle, noDistance));
	}
	/**
	 *
	 */
	std::shared_ptr<AbstractPercept> OdometerCompass::getPerceptFor(std::shared_ptr<AbstractStimulus> anAbstractStimulus) const {
		Robot *robot = dynamic_cast<Robot*>(agent);
		if (robot) {
			wxPoint robotLocation = robot->getPreviousPosition();

			DistanceStimulus *distanceStimulus = dynamic_cast<DistanceStimulus*>(anAbstractStimulus.get());
			if (distanceStimulus) {
				if (distanceStimulus->distance == noDistance) {
					return (std::make_shared<OrientationPercept>(invalidDistance, invalidDistance, wxPoint(invalidDistance, invalidDistance)));
				}
				wxPoint endpoint { static_cast<int>(robotLocation.x + std::cos(distanceStimulus->angle) * distanceStimulus->distance), static_cast<int>(robotLocation.y + std::sin(distanceStimulus->angle) * distanceStimulus->distance) };

				return (std::make_shared<OrientationPercept>((double) distanceStimulus->angle, (double) distanceStimulus->distance, endpoint));
			}
		}

		return (std::make_shared<OrientationPercept>(invalidDistance, invalidDistance, wxPoint(invalidDistance, invalidDistance)));
	}
	/**
	 *
	 */
	std::string OdometerCompass::asString() const {
		return "OdometerCompass";
	}
	/**
	 *
	 */
	std::string OdometerCompass::asDebugString() const {
		return asString();
	}
} // namespace Model
