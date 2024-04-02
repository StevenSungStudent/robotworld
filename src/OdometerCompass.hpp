/*
 * OdometerCompass.hpp
 *
 *  Created on: Oct 4, 2023
 *      Author: steven
 */

#ifndef SRC_ODOMETERCOMPASS_HPP_
#define SRC_ODOMETERCOMPASS_HPP_
#include <random>

#include "Config.hpp"
#include "AbstractSensor.hpp"
#include "OrientationPercept.hpp"
#include "DistanceStimulus.hpp"

namespace Model
{
	/**
	 * Compile time configurable length of the laser beam
	 */
//	const short int laserBeamLength = 1024;

	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	/**
	 *
	 */
	class OdometerCompass : public AbstractSensor
	{
		public:
		/**
		 *
		 */
			/**
			 *
			 */
			explicit OdometerCompass( Robot& aRobot);
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;
			/**
			 *
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;

			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}
		protected:
		private:
			/**
			 * Standard deviation of the odometer per 10 pixels
			 */
			static double stddevCompass;
			static double stddevOdometer;
	};
} // namespace Model

#endif /* SRC_ODOMETERCOMPASS_HPP_ */
