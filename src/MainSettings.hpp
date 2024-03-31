#ifndef MAINSETTINGS_HPP_
#define MAINSETTINGS_HPP_

#include "Config.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <string>

namespace Application {

	/*
	 *
	 */
	class MainSettings {
	public:
		/**
		 *
		 */
		MainSettings();
		/**
		 *
		 */
		virtual ~MainSettings();
		/**
		 *
		 */
		bool getDrawOpenSet() const;
		/**
		 *
		 */
		void setDrawOpenSet(bool aDrawOpenSet);
		/**
		 *
		 */
		unsigned long getSpeed() const;
		/**
		 *
		 */
		void setSpeed(unsigned long aSpeed);
		/**
		 *
		 */
		unsigned long getWorldNumber() const;
		/**
		 *
		 */
		void setWorldNumber(unsigned long aWorldNumber);

		double getCompassError() const {
			return compassError;
		}

		double getLidarDegrees() const {
			return lidarDegrees;
		}

		double getLidarError() const {
			return lidarError;
		}

		double getOdometerError() const {
			return odometerError;
		}

		bool isKalmanActive() const {
			return kalmanActive;
		}

		void setKalmanActive(bool kalmanActive) {
			this->kalmanActive = kalmanActive;
		}

		bool isParticleFilterActive() const {
			return particleFilterActive;
		}

		void setParticleFilterActive(bool particleFilterActive) {
			this->particleFilterActive = particleFilterActive;
		}

	private:
		bool drawOpenSet;
		unsigned long speed;
		unsigned long worldNumber;
		double compassError;
		double odometerError;
		double lidarError;
		double lidarDegrees;

		bool kalmanActive;
		bool particleFilterActive;

		void readConfiguration(const std::string &fileName);
	};

} /* namespace Application */

#endif /* SRC_MAINSETTINGS_HPP_ */
