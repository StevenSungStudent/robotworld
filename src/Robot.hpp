#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "Config.hpp"

#include "AbstractAgent.hpp"
#include "AStar.hpp"
#include "BoundedVector.hpp"
#include "DistancePercepts.hpp"
#include "Message.hpp"
#include "MessageHandler.hpp"
#include "Observer.hpp"
#include "Point.hpp"
#include "Region.hpp"
#include "Size.hpp"
#include "Matrix.hpp"
#include "OrientationPercept.hpp"
#include "ParticleFilter.hpp"

#include <utility>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace Messaging
{
	class Message;
	class Server;
	typedef std::shared_ptr< Server > ServerPtr;
}

namespace Model
{
	class Robot;
	typedef std::shared_ptr< Robot > RobotPtr;

	class Goal;
	typedef std::shared_ptr< Goal > GoalPtr;

	struct believedValue{
		double believedAngle;
		double believedDistance;
		wxPoint believedPoint;
	};

	/**
	 *
	 */
	class Robot :	public AbstractAgent,
					public Messaging::MessageHandler,
					public Base::Observer
	{
		public:
			/**
			 *
			 */
			Robot();
			/**
			 *
			 */
			explicit Robot( const std::string& aName);
			/**
			 *
			 */
			Robot(	const std::string& aName,
					const wxPoint& aPosition);
			/**
			 *
			 */
			std::vector<wxPoint> believedPosition;

			/**
			 *
			 */
			std::vector<believedValue> believedList;
			/**
			 *
			 */
			virtual ~Robot();
			/**
			 *
			 */
			std::string getName() const
			{
				return name;
			}
			/**
			 *
			 */
			void setName(	const std::string& aName,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			wxSize getSize() const;
			/**
			 *
			 */
			void setSize(	const wxSize& aSize,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			wxPoint getPosition() const
			{
				return (position);
			}

			wxPoint getPreviousPosition() const
			{
				return (previousPosition.at(previousPosition.size() - 2));
			}

			wxPoint getBelievedPosition() const
			{
				return (believedPosition.back());
			}

			wxPoint getBelievedPreviousPosition() const
			{
				return (believedPosition.at(believedPosition.size() - 2));
			}
			/**
			 *
			 */
			void setPosition(	const wxPoint& aPosition,
								bool aNotifyObservers = true);
			/**
			 *
			 */
			BoundedVector getFront() const;
			/**
			 *
			 */
			void setFront(	const BoundedVector& aVector,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			float getSpeed() const;
			/**
			 *
			 */
			void setSpeed( 	float aNewSpeed,
							bool aNotifyObservers = true);
			/**
			 *
			 * @return true if the robot is acting, i.e. either planning or driving
			 */
			bool isActing() const
			{
				return acting;
			}
			/**
			 *
			 */
			virtual void startActing() override;
			/**
			 *
			 */
			virtual void stopActing() override;
			/**
			 *
			 * @return true if the robot is driving
			 */
			bool isDriving() const
			{
				return driving;
			}
			/**
			 *
			 */
			virtual void startDriving();
			/**
			 *
			 */
			virtual void stopDriving();
			/**
			 *
			 * @return true if the robot is communicating, i.e. listens with an active ServerConnection
			 */
			bool isCommunicating() const
			{
				return communicating;
			}
			/**
			 * Starts a ServerConnection that listens at port 12345 unless given
			 * an other port by specifying a command line argument -local_port=port
			 */
			void startCommunicating();
			/**
			 * Connects to the ServerConnection that listens at port 12345 unless given
			 * an other port by specifying a command line argument -local_port=port
			 * and sends a message with messageType "1" and a body with "stop"
			 *
			 */
			void stopCommunicating();
			/**
			 *
			 */
			wxRegion getRegion() const;
			/**
			 *
			 */
			bool intersects( const wxRegion& aRegion) const;
			/**
			 *
			 */
			wxPoint getFrontLeft() const;
			/**
			 *
			 */
			wxPoint getFrontRight() const;
			/**
			 *
			 */
			wxPoint getBackLeft() const;
			/**
			 *
			 */
			wxPoint getBackRight() const;
			/**
			 * @name Observer functions
			 */
			//@{
			/**
			 * A Notifier will call this function if this Observer will handle the notifications of that
			 * Notifier. It is the responsibility of the Observer to filter any events it is interested in.
			 *
			 */
			virtual void handleNotification() override;
			//@}
			/**
			 *
			 */
			PathAlgorithm::OpenSet getOpenSet() const
			{
				return astar.getOpenSet();
			}
			/**
			 *
			 */
			PathAlgorithm::Path getPath() const
			{
				return path;
			}
			/**
			 * @name Messaging::MessageHandler functions
			 */
			//@{
			/**
			 * This function is called by a ServerSesssion whenever a message is received. If the request is handled,
			 * any response *must* be set in the Message argument. The message argument is then echoed back to the
			 * requester, probably a ClientSession.
			 *
			 * @see Messaging::RequestHandler::handleRequest( Messaging::Message& aMessage)
			 */
			virtual void handleRequest( Messaging::Message& aMessage);
			/**
			 * This function is called by a ClientSession whenever a response to a previous request is received.
			 *
			 * @see Messaging::ResponseHandler::handleResponse( const Messaging::Message& aMessage)
			 */
			virtual void handleResponse( const Messaging::Message& aMessage);
			//@}
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

			/**
			 * @name Variables for painting the sensor activity on the screen
			 */
			//@{
			// The start point of this run
			wxPoint startPosition;

			// Radar
			PointCloud currentRadarPointCloud; // The latest radar point cloud
			//@}

			// lidar
			PointCloud currentLidarPointCloud; // The latest lidar point cloud
			//@}

			// particle filter
			ParticleFilter particleFilter;
			//@}

		protected:
			/**
			 *
			 */
			void drive();
			/**
			 *
			 */
			void calculateRoute( GoalPtr aGoal);
			/**
			 *
			 */
			bool arrived( GoalPtr aGoal);
			/**
			 *
			 */
			bool collision();
			/**
			 *
			 */
			void calculateBelieve(OrientationPercept *orientation);
			/**
			 *
			 */
//			void generateParticles(const unsigned long& amount);


		private:
			/**
			 *
			 */
//			Matrix<double, 2, 2> covarianceMatrix { {0.0349066^2, 0}, {0, 0.1* 30} };
			Matrix<double, 2, 2> covarianceMatrix { {0.001218471, 0}, {0, 1} };
//			Matrix<double, 2, 2> covarianceMatrix { {1	, 0}, {0, 1} };
			/**
			 *
			 */
			std::string name;
			/**
			 *
			 */
			wxSize size;
			/**
			 *
			 */
			wxPoint position;
			/**
			 *
			 */
			std::vector<wxPoint> previousPosition;
			/**
			 *
			 */
			BoundedVector front;
			/**
			 *
			 */
			float speed;
			/**
			 *
			 */
			GoalPtr goal;
			/**
			 *
			 */
			PathAlgorithm::AStar astar;
			/**
			 *
			 */
			PathAlgorithm::Path path;
			/**
			 *
			 */
			bool acting;
			/**
			 *
			 */
			bool driving;
			/**
			 *
			 */
			bool communicating;
			/**
			 *
			 */
			std::thread robotThread;
			/**
			 *
			 */
			mutable std::recursive_mutex robotMutex;
			/**
			 *
			 */
			Messaging::ServerPtr server;
	};
} // namespace Model
#endif // ROBOT_HPP_
