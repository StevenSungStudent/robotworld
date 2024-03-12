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
//		long lastX;
//		long lastY;
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
//			/**
//			 *
//			 */
//			static void setStdDev(double aStdDev) {OdometerCompass::stddev = aStdDev;}
//			/**
//			 *
//			 */
//			static double getStdDev(){ return stddev;}
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
//
//class OdometerCompass : Model::AbstractSensor{
//public:
//	OdometerCompass();
//	virtual ~OdometerCompass();
//
//	long getNorth(const long &angle);
//	std::pair<long, long> getDistance(const long &lastX, const long &lastY);
//
//	/**
//	 * Get the raw measurements. This typically is done in the low level driver.
//	 */
//	std::shared_ptr< Model::AbstractStimulus > getStimulus();
//	/**
//	 * Translate the stimulus into a percept. This typically is done in the high level driver.
//	 */
//	std::shared_ptr< Model::AbstractPercept > getPerceptFor( std::shared_ptr< Model::AbstractStimulus > anAbstractPercepts);
//};

#endif /* SRC_ODOMETERCOMPASS_HPP_ */
