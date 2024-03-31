#include "MainSettings.hpp"
#include <iostream>

namespace Application
{
	/**
	 *
	 */
	MainSettings::MainSettings() : drawOpenSet(true), speed(10), worldNumber(0), compassError(0), odometerError(0), lidarError(0)
	{
		readConfiguration("configuration.json");
	}
	/**
	 *
	 */
	MainSettings::~MainSettings()
	{
	}
	/**
	 *
	 */
	bool MainSettings::getDrawOpenSet() const
	{
		return drawOpenSet;
	}
	/**
	 *
	 */
	void MainSettings::setDrawOpenSet( bool aDrawOpenSet)
	{
		drawOpenSet = aDrawOpenSet;
	}
	/**
	 *
	 */
	unsigned long MainSettings::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void MainSettings::setSpeed( unsigned long aSpeed)
	{
		speed = aSpeed;
	}
	/**
	 *
	 */
	unsigned long MainSettings::getWorldNumber() const
	{
		return worldNumber;
	}
	/**
	 *
	 */
	void MainSettings::setWorldNumber( unsigned long aWorldNumber)
	{
		worldNumber = aWorldNumber;
	}

	void MainSettings::readConfiguration(const std::string& fileName){
		boost::property_tree::ptree root;
		boost::property_tree::read_json(fileName, root);

		compassError = root.get<double>("compass error");
		odometerError = root.get<double>("odometer error");
		lidarError = root.get<double>("lidar error");
		lidarDegrees = root.get<double>("lidar degrees");
	}
} /* namespace Application */
