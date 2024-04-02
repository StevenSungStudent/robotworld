#include "LaserDistanceSensor.hpp"

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"

#include <random>

namespace Model
{
	/**
	 *
	 */
	/* static */ double LaserDistanceSensor::stddev = 10.0;
	/**
	 *
	 */
	LaserDistanceSensor::LaserDistanceSensor( Robot& aRobot) :
								AbstractSensor( aRobot)
	{
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractStimulus > LaserDistanceSensor::getStimulus() const
	{
		Robot* robot = dynamic_cast<Robot*>(agent);
		if(robot)
		{
			static std::random_device rd{};
			static std::mt19937 gen{rd()};
		    std::normal_distribution<> noise{0,LaserDistanceSensor::stddev};

			double angle = Utils::Shape2DUtils::getAngle( robot->getFront());
			std::vector< WallPtr > walls = RobotWorld::getRobotWorld().getWalls();

			wxPoint robotLocation = robot->getPosition();

			wxPoint intersection{-1,-1};
			for (std::shared_ptr< Wall > wall : walls)
			{
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				wxPoint laserEndpoint{	static_cast<int>(robotLocation.x + std::cos( angle) * laserBeamLength + noise(gen)) ,
										static_cast<int>(robotLocation.y + std::sin( angle) * laserBeamLength + noise(gen))};

				wxPoint currentIntersection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, robotLocation, laserEndpoint);

				if (currentIntersection != wxDefaultPosition)
				{
					if(intersection == wxDefaultPosition)
					{
						intersection = currentIntersection;
					}else if(Utils::Shape2DUtils::distance(robotLocation,currentIntersection) < Utils::Shape2DUtils::distance(robotLocation,intersection))
					{
						intersection = currentIntersection;
					}
				}
			}
			if(intersection != wxDefaultPosition)
			{
				double distance = Utils::Shape2DUtils::distance(robotLocation,intersection);
				return std::make_shared< DistanceStimulus >( angle,distance);
			}
		}
		return std::make_shared< DistanceStimulus >( noAngle,noDistance);
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept > LaserDistanceSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		Robot* robot = dynamic_cast< Robot* >( agent);
		if (robot)
		{
			wxPoint robotLocation = robot->getPosition();

			DistanceStimulus* distanceStimulus = dynamic_cast< DistanceStimulus* >( anAbstractStimulus.get());
			if(distanceStimulus)
			{
				if (std::fabs(distanceStimulus->distance - noDistance) <= std::numeric_limits<float>::epsilon())
				{
					return std::make_shared<DistancePercept>( wxPoint(noObject,noObject));
				}
				wxPoint endpoint{	static_cast< int >( robotLocation.x + std::cos( distanceStimulus->angle)*distanceStimulus->distance),
								static_cast< int >( robotLocation.y + std::sin( distanceStimulus->angle)*distanceStimulus->distance)};

				return std::make_shared<DistancePercept>( endpoint);
			}
		}

		return std::make_shared<DistancePercept>( wxPoint(invalidDistance,invalidDistance));
	}
	/**
	 *
	 */
	std::string LaserDistanceSensor::asString() const
	{
		return "LaserDistanceSensor";
	}
	/**
	 *
	 */
	std::string LaserDistanceSensor::asDebugString() const
	{
		return asString();
	}
} // namespace Model
