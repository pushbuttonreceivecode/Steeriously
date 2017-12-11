#ifndef STEERIOUSLY_HPP
#define STEERIOUSLY_HPP

/**
 *
 * @mainpage Steeriously
 * @version 0.1
 * @author Mark Richards
 * @date 12/9/2017
 * <br/>
 * \brief Steeriously. A dead-simple pure C++ library for adding steering behaviors to your game and multimedia entities.
 * <br />All credit goes to Mat Buckland for writing "Programming AI by Example" and providing such great steering code.
 * <br />All source is derived from "Programming AI by Example".<br />
 * <br />I can't recommend the book enough - you can buy the book here:
 * <br /><a href="http://www.amazon.com/Programming-Example-Wordware-Developers-Library/dp/1556220782/ref=sr_1_1?ie=UTF8&qid=1446057032&sr=8-1&keywords=programming+ai+by+example" target="_blank">Programming AI by Example</a>
 * <br />
 * <br />Steeriously has one humble goal - to make steering behavior accessible and fun to program.
 * <br />One way to achieve this is by redesigning the original code to expose a more general purpose API.
 * <br />
 * <br/>Tutorials and examples for using this library are available at:
 * <br/>
 * <br/><a href="http://pushbuttonreceivecode.com" target="_blank"> pushbuttonreceivecode.com </a>
 *
 * \par ===================== LICENSE =====================
 * The zlib/libpng License <br />
 * Copyright (c) 2017 Mark Richards <br />
 * <br />
 * This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
 * <br />
 *  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 *  2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 *  3. This notice may not be removed or altered from any source distribution.
**/

#include <vector>
#include <memory>
#include <iostream>

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/GeometryHelpers.hpp>
#include <steeriously/Path.hpp>
#include <steeriously/Transformations.hpp>
#include <steeriously/Utilities.hpp>
#include <steeriously/Vector2.hpp>
#include <steeriously/VectorMath.hpp>
#include <steeriously/Wall.hpp>

/**
*\brief All steering functions are templated for easy application
* to objects inheriting from Agent. Because of this, it is easy
* to add into any type of design - be it a class hierarchy or
* a more advanced data structure or system (such as an entity
* component system). All functions return a steer::steer::Vector2 that
* can be used to calculate a force with which the entity should
* be moved given each type of behavior. Combine each calculated
* force with a weight (any scalar value - but keep it sane or
* behavior could become strange). Simply multiply each weight,
* or weights, by the force generated by each behavior's return
* value to fine-tune it. In addition to the steering functions
* themselves, this file contains various utilities used by them
* to perform their work.
**/

namespace steer //begin steeriously namespace
{
    /**
	* \brief The following functions are utilities
	* used by the main template functions to perform
	* their calculations.
	**/

	/**
	* \fn	template<class T, class N>
	*		steer::Vector2 calculateTarget(const T& agent, const N& other);
	* \brief Template function to compute a target given another agent.
	* \param agent - a steer::Agent derived object.
	* \param other - a steer::Agent derived object.
	**/
	template<class T, class N>
	steer::Vector2 calculateTarget(const T& agent, const N& other)
	{
		steer::Vector2 to = other->getPosition() - agent->getPosition();

		//constrains processing to those cases where
		//the other agent is within range
		if (VectorMath::lengthSquared(to) > agent->getThreatRange() * agent->getThreatRange())
            return steer::Vector2(0.0, 0.0);

		//find the time in the future to extrapolate to
		//as a function of the distance between the agents
		//and their speeds
		float extrapolateTime = VectorMath::length(to) / (agent->getMaxSpeed() + other->getSpeed());

		//target acquired!
		steer::Vector2 target = steer::Vector2(other->getPosition() + other->getVelocity() * extrapolateTime);
		return target;
	}

	/**
	* \fn	template<class T, class N, class P>
	*		steer::Vector2 calculateTarget(const T& agent, const N& otherA, const P& otherB);
	* \brief Template function to compute a target between two agents.
	* \param agent - a steer::Agent  derivedobject.
	* \param otherA - a steer::Agent derived object.
	* \param otherB - a steer::Agent derived object.
	**/
	template<class T, class N, class P>
	steer::Vector2 calculateTarget(const T& agent, const N& otherA, const P& otherB)
	{
		//find the midpoint between the two other agents
		steer::Vector2 mid = (otherA->getPosition() + otherB->getPosition()) / 2.f;

		//find the time in the future to extrapolate to
		//as a function of the distance between the agent
		//and the midpoint in terms of the max speed
		float extrapolateTime = VectorMath::distance(agent->getPosition(), mid) / agent->getMaxSpeed();

		//extrapolate position based on time in the future
		//assuming the other agents are moving in a straight line
		steer::Vector2 posA = otherA->getPosition() + otherA->getVelocity() * extrapolateTime;
		steer::Vector2 posB = otherB->getPosition() + otherB->getVelocity() * extrapolateTime;

		//midpoint found - voila!
		mid = (posA + posB) / 2.f;

		return mid;
	}

    /**
	* \fn	template <class T>
	*		steer::Vector2 Seek(const T* agent);
	* \brief A function returning the value necessary to steer an agent towards a target.
	* \param agent - a steer::Agent derived object.
	**/
	template <class T>
	steer::Vector2 Seek(const T& agent)
	{
		steer::Vector2 velocity = { 0.0, 0.0 };
		if (agent != nullptr)
		{
			velocity = VectorMath::normalize(agent->getTarget() - agent->getPosition()) * agent->getMaxSpeed();
			return (velocity - agent->getVelocity());
		}
		else
			return velocity;
	};

	/**
	* \fn	steer::Vector2 Seek(const T& agent, steer::Vector2 target);
	* \brief A function returning the value necessary to steer an agent towards a target.
	* \param agent - a steer::Agent derived object.
	* \param target - the target vector you are trying to reach.
	**/
	template <class T>
	steer::Vector2 Seek(const T& agent, steer::Vector2 target)
	{
		steer::Vector2 velocity = { 0.0, 0.0 };

		velocity = VectorMath::normalize(target - agent->getPosition()) * agent->getMaxSpeed();

		return (velocity - agent->getVelocity());
	};

	/**
	* \fn   template<class T, class conT>
	*       steer::Vector2 Alignment(const T& agent, const conT& neighbors);
	* \brief Template function for keeping groups of agents in alignment.
	* \param agent - a steer::Agent derived object.
	* \param neighbors - a std::vector container of steer::Agent derived objects.
	**/
	template<class T, class conT>
	steer::Vector2 Alignment(const T& agent, const conT& neighbors)
	{
		steer::Vector2 avg = steer::Vector2(0.0, 0.0);
		int count = 0;

		if (agent != nullptr)
		{
		    for (auto& i : neighbors)
			{
				//exclude agent of interest, make sure
				//neighboring agents are tagged in the group
				if (i != nullptr && i != agent && i->taggedInGroup())
				{
					//summing the heading vectors of
					//agents tagged in the group
					avg += i->getHeading();

					++count;
				}
			}

			//only get avg if there are agents around
			if (count > 0)
			{
				avg /= (float)count;

				avg -= agent->getHeading();
			}
		}

		return avg;
	}

	/**
	* \fn   template <class T, class conT>
	*		steer::Vector2 Separation(const T& agent, const conT& neighbors);
	* \brief Template function for keeping groups of agents from clumping together.
	* \param agent - a steer::Agent derived object.
	* \param neighbors - a std::vector container of steer::Agent derived objects.
	**/
	template <class T, class conT>
	steer::Vector2 Separation(const T& agent, const conT& neighbors)
	{
		steer::Vector2 force = steer::Vector2(0.0, 0.0);
		steer::Vector2 toTarget = steer::Vector2(0.0, 0.0);
		steer::Vector2 normal = steer::Vector2(0.0, 0.0);
		float length = 0;

		if (agent != nullptr)
		{
		    for (auto& i : neighbors)
			{
				//exclude agent of interest and make
				//sure neighboring agents are tagged in the group
				if (i != nullptr && i != agent && i->taggedInGroup())
				{
					toTarget = agent->getPosition() - i->getPosition();

					normal = VectorMath::normalize(toTarget);
					length = VectorMath::length(toTarget);

					//calculate a force as a function
					//of its distance from each neighboring agent
					force += normal / length;
				}
			}
		}

		return force;
	}

	/**
	* \fn	template <class T, class conT>
	*		steer::Vector2 Cohesion(const T& agent, const conT& neighbors);
	* \brief Template function for keeping groups of agents cohesive.
	* \param agent - a steer::Agent derived object.
	* \param neighbors - a std::vector container of steer::Agent derived objects.
	**/
	template <class T, class conT>
	steer::Vector2 Cohesion(const T& agent, const conT& neighbors)
	{
		steer::Vector2 centerOfMass = steer::Vector2(0.0, 0.0);
		steer::Vector2 force = steer::Vector2(0.0, 0.0);

		int count = 0;

		if (agent != nullptr)
		{
		    for (auto& i : neighbors)
			{
				//exclude agent of interest and make sure
				//nighboring agents are tagged in the group
				if (i != nullptr && i != agent && i->taggedInGroup())
				{
					//sum their positions
					centerOfMass += i->getPosition();

					++count;
				}
			}

			if (count > 0)
			{
				//compute the center of mass
				centerOfMass /= (float)count;

				//calculate the force with Seek(T& agent, steer::Vector2 target) overload
				//internally, this does the same thing without
				//setting m_target - which is undesirable
				//when tuning flocking behavior
				force = Seek(agent, centerOfMass);
			}

			//normalize due to the fact that
			//cohesion generally factors in
			//higher than separation and alignment
			force = VectorMath::normalize(force);
		}

		return force;
	};

	/**
	* \fn	template<class T, Uint32>
	*		steer::Vector2 Arrive(const T& agent, Uint32 deceleration);
	* \brief A function returning the value necessary to steer an agent towards a target with a smooth arrival.
	* \param agent - a steer::Agent derived object.
	* \param deceleration - an unsigned integer, SEE BEHAVIORHELPERS.HPP.
	**/
	template<class T>
	steer::Vector2 Arrive(const T& agent, Uint32 deceleration)
	{
		steer::Vector2 target = agent->getTarget() - agent->getPosition();

		//calculate the distance to the target
		float distance = VectorMath::length(target);

		//bail if on target
		if (distance > 0)
		{
			//speed is relative to distance and deceleration factors
			float speed = distance / ((float)deceleration * agent->getDecelerationTweaker());

			//cap speed at agent's max
			speed = std::min(speed, agent->getMaxSpeed());

			//voila!
			steer::Vector2 velocity = target * speed / distance;

			return (velocity - agent->getVelocity());
		}

		//nothing to do - stay the course!
		return steer::Vector2(0.0, 0.0);
	}

	/**
	* \fn	template<class T, class N>
	*		steer::Vector2 calculatePursuit(const T& agent, const N& evader);
	* \brief A method returning the value necessary for an agent to pursue a target.
	* \param agent - a steer::Agent derived object.
	* \param evader - a steer::Agent derived object.
	**/
	template<class T, class N>
	steer::Vector2 Pursuit(const T& agent, const N& evader)
	{
		steer::Vector2 to = evader->getPosition() - agent->getPosition();

		float heading = VectorMath::dotProduct(agent->getHeading(), evader->getHeading());

		//evading agent is ahead, so seek to it
		if ((VectorMath::dotProduct(to, agent->getHeading()) > 0) && (heading < -0.95))//acos(0.95)=18 degs
		{
			agent->setTarget(evader->getPosition());
			return Seek<T>(agent);
		}

		//evading agent is not ahead
		//...extrapolate its future position
		float extrapolationTime = VectorMath::length(to) / (agent->getMaxSpeed() + evader->getSpeed());

		//seek...
		agent->setTarget(evader->getPosition() + evader->getVelocity() * extrapolationTime);
		return Seek<T>(agent);
	}

	/**
	* \fn	template<class T, class N>
	*		steer::Vector2 offsetPursuit(const T& agent, const N& leader, const steer::BehaviorParameters& parameters);
	* \brief This method returns a vector necessary to maintain a position in the direction of offset from the target vehicle.
	* \param agent - a steer::Agent derived object.
	* \param leader - a steer::Agent derived object.
	* \param parameters - a steer::BehaviorParameters objects.
	**/
	template<class T, class N>
	steer::Vector2 OffsetPursuit(const T& agent, const N& leader, const steer::BehaviorParameters& parameters)
	{
		//thisAgent->setTarget(calculateTarget(thisAgent, leader));
		agent->setTarget(leader->getPosition());

		steer::Vector2 to = agent->getTarget() - agent->getPosition();

		//distance to target
		float distance = VectorMath::length(to);

		if (distance > 0)
		{
			//speed is relative to distance and deceleration factors
			float speed = distance / ((float)parameters.deceleration * agent->getDecelerationTweaker());

			//cap velocity at max
			speed = std::min(speed, agent->getMaxSpeed());

			//seek...
			steer::Vector2 velocity = to * speed / distance;

			return (velocity - agent->getVelocity());
		}

		return steer::Vector2(0.0, 0.0);
	}

	/**
	* \fn	template<class T>
	*		steer::Vector2 Flee(const T& agent);
	* \brief A function returning the value necessary to steer an agent away from a target.
	* \param agent - a steer::Agent derived object.
	**/
	template<class T>
	steer::Vector2 Flee(const T& agent)
	{
		steer::Vector2 velocity = VectorMath::normalize(agent->getPosition() - agent->getTarget()) * agent->getMaxSpeed();
		if (VectorMath::distanceSquared(agent->getPosition(), agent->getTarget()) > agent->getThreatRange()*agent->getThreatRange())
			return steer::Vector2(0.0, 0.0);
		else
			return (velocity - agent->getVelocity());
	}

	/**
	* \fn	template<class T, class T>
	*		steer::Vector2 Evade(const T& agent, const T& other);
	* \brief This function returns the value necessary for evading a pursuing agent.
	* \param agent - a steer::Agent derived object.
	* \param other - a steer::Agent derived object.
	**/
	template<class T, class N>
	steer::Vector2 Evade(const T& agent, const N& other)
	{
		agent->setTarget(calculateTarget<T,N>(agent, other));

		steer::Vector2 velocity = VectorMath::normalize(agent->getPosition() - agent->getTarget()) * agent->getMaxSpeed();
		if (VectorMath::distanceSquared(agent->getPosition(), agent->getTarget()) > agent->getThreatRange()*agent->getThreatRange())
			return steer::Vector2(0.0, 0.0);
		else
			return (velocity - agent->getVelocity());
	}

	/**
	\fn steer::Vector2 findPosition(const steer::Vector2& goalPosition, const float radius, const steer::Vector2& avoidPosition, float distanceBuffer);
	\brief This function calculates a position located on the other side of some position in space, given a radius, that is out of reach from an undesirable position (for example, a pursuing agent).
	\param goalPosition - a 2D vector of doubles.
	\param radius - a plain old float.
	\param avoidPosition - a 2D vector of doubles.
	\param boundary - a plain old float.
	**/
	inline steer::Vector2 findPosition(const steer::Vector2& goalPosition, const float radius, const steer::Vector2& avoidPosition, float distanceBuffer)
	{
		//calculate the desired buffer between the goal position
		//and the radius of the goal area
		float       away = radius + distanceBuffer;

		//calculate the heading toward the avoidance position
		steer::Vector2 to = VectorMath::normalize(goalPosition - avoidPosition);

		//calculate the position relative to the goal and
		//scaled according to the direction of the avoidance
		//position and the distance *from* the goal
		return (to * away) + goalPosition;
	}

	/**
	* \fn	template<class T, class N, class conT>
	*		steer::Vector2 Hide(const T& agent, const N& other, const conT& obstacles, const steer::BehaviorParameters& parameters);
	* \brief Given another agent position to hide from and a list of steer::Bases, this method attempts to put an obstacle between itself and its opponent.
	* \param agent - a steer::Agent derived object.
	* \param other - a steer::Agent derived object.
	* \param obstacles - a std::vector of steer::Base derived objects.
	* \param parameters - a steer::BehaviorParameters object.
	**/
	template<class T, class N, class conT>
	steer::Vector2 Hide(const T& agent, const N& other, const conT& obstacles, const steer::BehaviorParameters& parameters)
	{
		float closest = steer::MaxFloat;
		steer::Vector2 best;

		for (auto& ob : obstacles)
		{
			//find a hiding spot, given each obstacle
			steer::Vector2 spot = findPosition(ob->getPosition(), ob->getRadius(), other->getPosition(), agent->getDistanceBuffer());

			//determine the closest hiding spot
			float distance = steer::VectorMath::distanceSquared(spot, agent->getPosition());

			if (distance < closest)
			{
				closest = distance;

				best = spot;
			}

		}

		//no hiding spots...?
		//...evade the other agent
		if (closest == steer::MaxFloat)
		{
			return Evade<T, N>(agent, other);
		}

		//otherwise, arrive at the hiding spot
		agent->setTarget(best);
		return Arrive<T>(agent, parameters.deceleration);
	}

	/**
	* \fn	template<class T, class N, class P, Uint32>
	*		steer::Vector2 Interpose(const T& agent, const N& otherA, const P& otherB, const steer::BehaviorParameters& parameters);
	* \brief This method results in a target used to steer the agent to the center of the vector connecting two moving agents.
	* \param agent - a steer::Agent derived object.
	* \param otherA - a steer::Agent derived object.
	* \param otherB - a steer::Agent derived object.
	* \param parameters - a steer::BehaviorParameters object.
	**/
	template<class T, class N, class P>
	steer::Vector2 Interpose(const T& agent, const N& otherA, const P& otherB, const steer::BehaviorParameters& parameters)
	{
		//calculate target given two agents
		agent->setTarget(calculateTarget<T,N,P>(agent, otherA, otherB));

		steer::Vector2 to = agent->getTarget() - agent->getPosition();

		float distance = VectorMath::length(to);

		if (distance > 0)
		{
			float speed = distance / ((float)parameters.deceleration * agent->getDecelerationTweaker());

			//cap velocity at the max
			speed = std::min(speed, agent->getMaxSpeed());

			steer::Vector2 velocity = to * speed / distance;

			return (velocity - agent->getVelocity());
		}

		//otherwise, stay the course
		return steer::Vector2(0.0, 0.0);
	}

	/**
	* \fn steer::Vector2 Wander();
	* \brief This method makes the agent wander about randomly
	* \param thisAgent - a steer::Agent derived object.
	**/
	template<class T>
	steer::Vector2 Wander(const T& agent)
	{
		if (agent != nullptr)
		{
			//this behavior is dependent on the update rate, so this line must
            //be included when using time independent framerate.
            float JitterThisTimeSlice = agent->m_wanderJitter * agent->getElapsedTime();

            //first, add a small random vector to the target's position
            agent->m_wanderTarget += steer::Vector2(RandomClamped() * JitterThisTimeSlice, RandomClamped() * JitterThisTimeSlice);

            //reproject this new vector back on to a unit circle
            agent->m_wanderTarget = VectorMath::normalize(agent->m_wanderTarget);

            //increase the length of the vector to the same as the radius
            //of the wander circle
            agent->m_wanderTarget *= agent->m_wanderRadius;

            //move the target into a position WanderDist in front of the agent
            steer::Vector2 target = agent->m_wanderTarget + steer::Vector2(agent->m_wanderDistance, 0.0);

            //project the target into world space
            steer::Vector2 Target = PointToWorldSpace(target, agent->getHeading(), agent->getSide(), agent->getPosition());

            return Target - agent->getPosition();
		}
	}

	/**
	* \fn   template <class T, class conT>
	*		steer::Vector2 ObstacleAvoidance(const T& agent, const conT& obstacles, const steer::BehaviorParameters& parameters);
	* \brief This template method returns a steering force which will attempt to keep the agent away from any obstacles it may encounter.
	* \param agent - a steer::Agent derived object.
	* \param obstacles - a std::vector of conT obstacles.
	* \param parameters - a steer::BehaviorParameters object.
	**/
	template <class T, class conT>
	steer::Vector2 ObstacleAvoidance(const T& agent, const conT& obstacles, const steer::BehaviorParameters& parameters)
	{
	    //the detection box length is proportional to the agent's velocity
		agent->setBoxLength(parameters.MinDetectionBoxLength + (agent->getSpeed() / agent->getMaxSpeed()) * parameters.MinDetectionBoxLength);

        //tag obstacles...
        steer::TagObstaclesWithinViewRange< T, conT >(agent, obstacles, agent->m_boxLength);

		//closest obstacle
		steer::SphereObstacle* closest = nullptr;

		//track distance to closest obstacle
		float distance = steer::MaxDouble;

		//track position of closest obstacle
		steer::Vector2 position;

		for (auto& i : obstacles)
		{
			if (i != nullptr && i->taggedInGroup())
			{
				//get obstacle position
				steer::Vector2 local = PointToLocalSpace(i->getPosition(), agent->getHeading(), agent->getSide(), agent->getPosition());

				//obstacle is ahead the agent
				if (local.x >= 0)
				{
					//condition for potential intersection
					float radius = i->getRadius() + agent->getBoundingRadius();

					if (fabs(local.y) < radius)
					{
						//formula x = local.x +/-sqrt(r^2-local.y^2) , y=0
						//check for negative value indicating intersection
						float subtrahend = sqrt(radius*radius - local.y*local.y);

						float difference = local.x - subtrahend;

						//if difference is less than 0, use positive
						//subtrahend equation
						if (difference <= 0.0)
						{
							difference = local.x + subtrahend;
						}

						//update closest obstacle, its distance, and its position
						if (difference < distance)
						{
							distance = difference;

							closest = i;

							position = local;
						}
					}
				}
			}
		}

		steer::Vector2 force;

		if (closest != nullptr)
		{
			//scale avoidance force with respect
			//to the agent's distance from the obstacle
			float scale = 1.0 + (agent->boxLength() - position.x) / agent->boxLength();

			//calculate the lateral force
			force.y = (closest->getRadius() - position.y) * scale;

			const float brake = 0.2;

			//apply braking factor to the force
			force.x = (closest->getRadius() - position.x) * brake;
		}

		//convert to world space
		return VectorToWorldSpace(force, agent->getHeading(), agent->getSide());
	}

	/**
	* \fn	template<class T>
	*		steer::Vector2 WallAvoidance(const T& agent, const std::vector<steer::Wall*> walls);
	* \brief This method returns a steering force which will keep the agent away from any walls it may encounter.
	* \param agent - a steer::Agent derived object.
	* \param walls - a pointer to a std::vector of steer::Wall objects.
	**/
	template<class T>
	steer::Vector2 WallAvoidance(const T& agent, const std::vector<steer::Wall*>& walls)
	{
		//the feelers are contained in a std::vector
		agent->createFeelers();

		float distance = 0.0;
		float closestDistance = steer::MaxFloat;

		//this will hold an index into the vector of walls
		int closest = -1;

		steer::Vector2 force = steer::Vector2(0.0, 0.0);
		steer::Vector2 tempPoint = steer::Vector2(0.0, 0.0);
		steer::Vector2 point = steer::Vector2(0.0, 0.0);

		for (unsigned int flr = 0; flr<agent->getFeelers().size(); ++flr)
		{
			//for each feeler, check each wall for an intersection point
			for (unsigned int w = 0; w<walls.size(); ++w)
			{
				if (steer::LineIntersection2D(agent->getPosition(), agent->getFeelers()[flr], walls[w]->From(), walls[w]->To(), distance, tempPoint))
				{
					//keep a record of the intersection data
					if (distance < closestDistance)
					{
						closestDistance = distance;

						closest = w;

						point = tempPoint;
					}
				}
			}

			//calculate a force to steer away
			//from wall if there is an intersection
			if (closest >= 0)
			{
				//calculate the magnitude the agent
				//will overshoot the wall by
				steer::Vector2 over = agent->getFeelers()[flr] - point;

				//create a wall avoidance force
				//scaled by the overshoot
				force = walls[closest]->Normal() * steer::VectorMath::length(over);
			}

		}

		return force;
	}

	/**
	* \fn	template<class T>
	*		steer::Vector2 pathFollowing(const T& thisAgent, steer::Path& path, const steer::BehaviorParameters& params);
	* \brief Given a series of steer::Vector2's, this method produces a force that will move the agent along the way points in order.
	* \param agent - a steer::Agent derived object.
	* \param path - a steer::Path object.
	* \param params - a steer::BehaviorParameters object.
	**/
	template<class T>
	steer::Vector2 PathFollowing(const T& agent, steer::Path* path, const steer::BehaviorParameters& params)
	{
	    if(path != nullptr)
        {
            //continue on to the next waypoint in the path
            if (steer::VectorMath::distanceSquared(path->currentWaypoint(), agent->getPosition()) < params.waypointSeekDistanceSquared)
            {
                path->setNextWaypoint();
            }

            if (!path->finished())
            {
                agent->setTarget(path->currentWaypoint());
                return Seek< T >(agent);
            }

            else
            {
                agent->setTarget(path->currentWaypoint());
                return Arrive< T >(agent, agent->m_deceleration);
            }
        }
	}

} //end namespace steeriously

#endif //STEERIOUSLY_HPP
