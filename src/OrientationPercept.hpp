/*
 * OrientationStimulus.hpp
 *
 *  Created on: Oct 10, 2023
 *      Author: steven
 */

#ifndef SRC_ORIENTATIONSTIMULUS_HPP_
#define SRC_ORIENTATIONSTIMULUS_HPP_

#include "Config.hpp"

#include "DistanceStimulus.hpp"
#include "Point.hpp"

#include <limits>

namespace Model
{
	/**
	 *
	 */
	class OrientationPercept : public AbstractPercept
	{
		public:
			/**
			 *
			 */
			explicit OrientationPercept(const double& anAngle, const double& aDistance, const wxPoint& aPoint) :
				angle(anAngle), distance(aDistance), point(aPoint)
			{
			}

			double angle;
			double distance;
			wxPoint point;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override
			{
				return "OrientationPercept: " + std::to_string(angle) + ", " + std::to_string(distance) + ", " + std::to_string(point.x) + ", " + std::to_string(point.y);
			}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override
			{
				return asString();
			}
	}; //	class DistancePercept
} // namespace Model

#endif /* SRC_ORIENTATIONSTIMULUS_HPP_ */
