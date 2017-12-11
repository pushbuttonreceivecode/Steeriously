
#ifndef AGENT_HPP
#define AGENT_HPP

#include <vector>

#include <steeriously/BehaviorData.hpp>
#include <steeriously/Utilities.hpp>
#include <steeriously/Vector2.hpp>
#include <steeriously/VectorMath.hpp>

namespace steer //steeriously namespace
{

    /**
    *\class Agent
    *\brief The invisible, but highly necessary, automaton which drives your
    *game entity/graphical representation's motion. Getters and Setters are provided,
    *however it will always be easier to just use the data - since it is all public.
    **/
	class Agent
	{

	public:

		/**
		* \fn Agent();
		* \brief Default constructor
		**/
		Agent();

		/**
		\fn Agent(steer::Vector2 position, float radius, steer::Vector2 velocity, steer::Vector2 heading, steer::Vector2 side, float mass, float maxSpeed, float maxForce, float maxTurnRate);
		\brief Construct agent from values.
		\param position - a steer::Vector2 of floats.
		\param radius - a plain old float.
		\param velocity - a steer::Vector2 of floats.
		\param heading - a steer::Vector2 of floats.
		\param side - a steer::Vector2 of floats.
		\param mass - a plain old float.
		\param maxSpeed - a plain old float.
		\param maxForce - a plain old float.
		\param maxTurnRate - a plain old float.
		**/
		Agent(steer::Vector2 position, float radius, steer::Vector2 velocity, steer::Vector2 heading, steer::Vector2 side, float mass, float maxSpeed, float maxForce, float maxTurnRate);

		/**
		* \fn Agent(steer::BehaviorParameters* params);
		* \brief Construct agent from struct of parameters
		* \param params - a steer::BehaviorParameters object;
		**/
		Agent(steer::BehaviorParameters* params);

		/// Destructor
		virtual ~Agent();

		/**
		\fn steer::Vector2 getPosition() const;
		\brief Gets the position of the entity.
		**/
		steer::Vector2 getPosition() const { return m_agentPosition; };

		/**
		\fn void setPosition(steer::Vector2 newPosition);
		\brief Sets the position of the entity.
		\param newPosition - a steer::Vector2 of floats.
		**/
		void setPosition(steer::Vector2 newPosition) { m_agentPosition = newPosition; };

		/**
		\fn float getBoundingRadius() const;
		\brief Gets the bounding radius of the entity.
		**/
		float getBoundingRadius()const { return m_boundingRadius; }

		/**
		\fn void setBoundingRadius(float radius);
		\param radius - a plain old float.
		\brief Sets the bounding radius of the entity.
		**/
		void setBoundingRadius(float radius) { m_boundingRadius = radius; };

		/**
		\fn bool taggedInGroup() const;
		\brief Gets the tagged status (tagged in a group) of the entity.
		**/
		bool taggedInGroup() const { return m_tag; };

		/**
		\fn void Tag();
		\brief Tags the entity for an action.
		**/
		void Tag() { m_tag = true; };

		/**
		\fn void unTag();
		\brief Untags the entity.
		**/
		void unTag() { m_tag = false; };

		/**
		\fn steer::Vector2 getScale()const;
		\brief Returns the entity's scale.
		**/
		steer::Vector2 getScale() const { return m_scale; };

		/**
		\fn void setScale(steer::Vector2 val);
		\param val - a steer::Vector2 of floats.
		\brief Sets the entity's scale from an steer::Vector2.
		**/
		void setScale(steer::Vector2 val)
		{
			m_boundingRadius *= MaxOf(val.x, val.y) / MaxOf(m_scale.x, m_scale.y);
			m_scale = val;
		}

		/**
		\fn void setScale(float val);
		\param val - a plain old float.
		\brief Sets the entity's scale from a float.
		**/
		void setScale(float val)
		{
			m_boundingRadius *= (val / MaxOf(m_scale.x, m_scale.y));
			m_scale = steer::Vector2(val, val);
		};

		/**
		\fn steer::Vector2 getVelocity() const;
		\brief Gets the velocity of the agent.
		**/
		steer::Vector2 getVelocity() const { return m_velocity; }

		/**
		\fn float getMass() const;
		\brief Gets the mass of the agent. Mass is set on construction of the agent only.
		**/
		float getMass() const { return m_mass; }

		/**
		\fn steer::Vector2 getSide() const;
		\brief Gets the side vector (vector perpendicular to the direction the agent is heading) of the agent. <br />Side is set on construction of the agent only.
		**/
		steer::Vector2 getSide() const { return m_side; }

		/**
		\fn void setSide(const steer::Vector2 side);
		\param side - a steer::Vector2 of floats.
		\brief Sets the side vector (vector perpendicular to the direction the agent is heading) of the agent. <br />Side is set on construction of the agent only.
		**/
		void setSide(const steer::Vector2 side) { m_side = side; }

		/**
		\fn float getMaxSpeed() const;
		\brief Gets the max speed of the agent.
		**/
		float getMaxSpeed() const { return m_maxSpeed; }

		/**
		\fn void setMaxSpeed(float newSpeed);
		\brief Sets the velocity of the agent.
		\param newSpeed - a plain old float.
		**/
		void setMaxSpeed(float newSpeed) { m_maxSpeed = newSpeed; }

		/**
		\fn float getMaxForce() const;
		\brief Gets the max force of the agent.
		**/
		float getMaxForce() const { return m_maxForce; }

		/**
		\fn void setMaxForce(float maxForce);
		\brief Sets the max force of the agent.
		\param maxForce - a plain old float.
		**/
		void setMaxForce(float maxForce) { m_maxForce = maxForce; }

		/**
		\fn steer::Vector2 getForce() const;
		\brief Convenience method, gets the current steering force for the owner agent. Force is public data used for calculating steering force, therefore there is no setter.
		**/
		steer::Vector2 getForce() const { return m_steeringForce; }

		/**
		\fn bool speedMaxedOut() const;
		\brief Determines if the speed of the agent is maxed out or not.
		**/
		bool speedMaxedOut() const { return m_maxSpeed*m_maxSpeed >= VectorMath::lengthSquared(m_velocity); }

		/**
		\fn float getSpeed() const;
		\brief Gets the speed of the agent.
		**/
		float getSpeed() const { return VectorMath::length(m_velocity); }

		/**
		\fn float getSpeedSquared() const;
		\brief Gets the speed squared of the agent.
		**/
		float getSpeedSquared() const { return VectorMath::lengthSquared(m_velocity); }

		/**
		\fn steer::Vector2 getHeading() const;
		\brief Gets the heading of the agent.
		**/
		steer::Vector2 getHeading() const { return m_heading; }

		/**
		\fn void setHeading(steer::Vector2 newHeading);
		\brief Sets the heading of the agent.
		\param newHeading - a steer::Vector2 of floats.
		**/
		void setHeading(steer::Vector2 newHeading);

		/**
		\fn bool rotateHeadingToFacePosition(steer::Vector2 target);
		\brief Returns true when the heading is rotated to the target position, returns false otherwise.
		\param target - a steer::Vector2 of floats.
		**/
		bool rotateHeadingToFacePosition(steer::Vector2 target);

		/**
		\fn void getMaxTurnRate() const;
		\brief Gets the max turn rate of the agent.
		**/
		float getMaxTurnRate() const { return m_maxTurnRate; }

		/**
		\fn void setMaxTurnRate(float val);
		\brief Sets the max turn rate of the agent.
		\param maxTurnRate - a plain old float.
		**/
		void setMaxTurnRate(float maxTurnRate) { m_maxTurnRate = maxTurnRate; }

		/**
		\fn float getForwardComponent();
		\brief Calculates the component of the steering force that is parallel with the vehicle heading.
		**/
		float getForwardComponent();

		/**
		\fn float getSideComponent();
		\brief Calculates the component of the steering force that is perpendicular with the vehicle heading.
		**/
		float getSideComponent();

		/**
		\fn void setTarget(const steer::Vector2 target);
		\brief Sets the current target for the owner agent.
		\param target - a steer::Vector2 of floats.
		**/
		void setTarget(const steer::Vector2 target) { m_target = target; };

		/**
		\fn void getTarget() const;
		\brief Gets the current target for the owner agent.
		**/
		steer::Vector2 getTarget() const { return m_target; };

		/**
		* \fn void setThreatRange();
		* \brief Set the value of the range the object of interest must be in to trigger evasive action.
		* \param range - a plain old float.
		**/
		void setThreatRange(const float range) { m_threatRange = range; };

		/**
		* \fn float getThreatRange() const;
		* \brief Get the value of the range the object of interest must be in to trigger evasive action.
		**/
		float getThreatRange() const { return m_threatRange; };

		/**
		* \fn void setDecelerationTweaker(const float deceleration);
		* \brief Set the value for tweaking deceleration for the arrive component.
		* \param weight - a plain old float.
		**/
		void setDecelerationTweaker(const float deceleration) { m_decelerationTweaker = deceleration; };

		/**
		* \fn float getDecelerationTweaker() const;
		* \brief Get the value for tweaking deceleration for the arrive component.
		**/
		float getDecelerationTweaker() const { return m_decelerationTweaker; };

		/**
		* \fn void setWanderTarget(const steer::Vector2 target);
		* \brief Set the wander target.
		* \param target - a steer::Vector2 of floats.
		**/
		void setWanderTarget(const steer::Vector2 target) { m_wanderTarget = target; };

		/**
		* \fn steer::Vector2 getWanderTarget() const;
		* \brief Get the wander target.
		**/
		steer::Vector2 getWanderTarget() const { return m_wanderTarget; };

		/**
		* \fn void setWanderJitter(const float jitter);
		* \brief Set the wander jitter.
		* \param jitter - a plain old float.
		**/
		void setWanderJitter(const float jitter) { m_wanderJitter = jitter; };

		/**
		* \fn float getWanderJitter() const;
		* \brief Get the wander jitter.
		**/
		float getWanderJitter() const { return m_wanderJitter; };

		/**
		* \fn void setWanderRadius(const float radius);
		* \brief Set the radius that the agent will wander toward.
		* \param radius - a plain old float.
		**/
		void setWanderRadius(const float radius) { m_wanderRadius = radius; };

		/**
		* \fn float getWanderRadius()const;
		* \brief Get the radius the agent is wandering toward.
		**/
		float getWanderRadius() const { return m_wanderRadius; };

		/**
		* \fn void setWanderDistance(const float distance);
		* \brief Set the distance the agent will wander.
		* \param distance - a plain old float.
		**/
		void setWanderDistance(const float distance) { m_wanderDistance = distance; };

		/**
		* \fn float getWanderDistance() const;
		* \brief Get the distance the agent will wander.
		**/
		float getWanderDistance() const { return m_wanderDistance; };

		/**
		* \fn void setDistanceBuffer(const float buffer);
		* \brief Set the value for the buffer which dictates the minimum distance from a given area (for example, a hiding spot behind an obstacle).
		* \param buffer - a plain old float.
		**/
		void setDistanceBuffer(const float buffer) { m_distanceBuffer = buffer; };

		/**
		* \fn float getDistanceBuffer() const;
		* \brief Get the value for the distance buffer (minimum distance between self and a given area).
		**/
		float getDistanceBuffer() const { return m_distanceBuffer; };

		/**
		\fn void setOffset(const steer::Vector2 offset);
		\brief Sets the current steering offset for the owner agent - used for tweaking steering force.
		**/
		void setOffset(const steer::Vector2 offset) { m_offset = offset; };

		/**
		\fn steer::Vector2 getOffset() const;
		\brief Gets the current steering offset for the owner agent - \see setOffset(const steer::Vector2 offset);
		**/
		steer::Vector2 getOffset() const { return m_offset; }

		/**
		\fn void createFeelers();
		\brief A method for creating the feelers used for obstacle and wall avoidance;
		**/
		void createFeelers();

		/**
		\fn float boxLength() const;
		\brief Returns the value of box length used for obstacle and wall avoidance;
		**/
		float boxLength() const { return m_boxLength; }

		/**
		\fn void setBoxLength(const float length);
		\brief Returns the value of box length used for obstacle and wall avoidance;
		\param length - a plain old float.
		**/
		void setBoxLength(const float length) { m_boxLength = length; }

		/**
		\fn const std::vector<steer::Vector2>& getFeelers()const;
		\brief Returns a reference to the feelers used for obstacle and wall avoidance;
		**/
		const std::vector<steer::Vector2>& getFeelers() const { return m_feelers; }

		/**
		\fn virtual bool on(behaviorType behavior) = 0;
		\brief This pure virtual function tests if a specific bit of m_iFlags is set using bitwise operations. Must be overridden in derived classes.
		\param behavior - enum behaviorType.
		**/
		virtual bool on(steer::behaviorType behavior) = 0;

		/**
		\fn virtual void setSummingMethod(summingMethod sumMethod);
		\brief A virtual method used for setting the summing method for steering forces.
		\param sumMethod - an Uint32.
		**/
		virtual void setSummingMethod(Uint32 sumMethod);

		/**
		\fn  virtual bool accumulateForce(steer::Vector2 &startingForce, steer::Vector2 forceToAdd);
		\brief A virtual method used to accumulate forces from a combination of behaviors. Does nothing, override and implement in derived classes.
		\param startingForce - a steer::Vector2 of floats.
		\param forceToAdd - a steer::Vector2 of floats.
		**/
		virtual bool accumulateForce(steer::Vector2 &startingForce, steer::Vector2 forceToAdd);

		/**
		\fn virtual steer::Vector2 CalculateWeightedSum();
		\brief A virtual method that sums steering forces as a weighted sum. Does nothing, override and implement in derived classes.
		**/
		virtual steer::Vector2 calculateWeightedSum();

		/**
		\fn virtual steer::Vector2 CalculatePrioritized();
		\brief A virtual method that sums steering forces in prioritized order. Does nothing, override and implement in derived classes.
		**/
		virtual steer::Vector2 calculatePrioritized();

		/**
		\fn virtual steer::Vector2 Calculate();
		\brief A pure virtual method for calculating the steering vector.
		**/
		virtual steer::Vector2 Calculate() = 0;

		/**
		* \fn float getElapsedTime();
		* \brief Get the time elapsed since the last frame.
		**/
		float getElapsedTime() { return m_timeElapsed; };

		/**
		* \fn float setElapsedTime(float e);
		* \brief Set the time elapsed since the last frame.
		**/
		float setElapsedTime(float e) { m_timeElapsed = e; };

        //all data are public
        //getters/setters provided for anyone that wants them...
        //Since the whole purpose of this library is to
        //keep things easy - public data is the best approach
	public:
		steer::Vector2								m_agentPosition;///< The Entity's internal position value.
		steer::Vector2								m_scale;///< The Entity's internal scale value.
		bool										m_tag;/// Generic flag to indicate that the entity is flagged for some process.
		float										m_boundingRadius;///< The Entity's internal bounding radius value.
		steer::Vector2                              m_steeringForce;///< For calculating the steering force internally from all combined behaviors.
		float                                       m_waypointSeekDistanceSquared;///< the distance (squared) a vehicle has to be from a path waypoint before it starts seeking to the next waypoint
		std::vector<steer::Vector2>                 m_feelers;///< a vertex buffer to contain the feelers for wall avoidance
		steer::Vector2                              m_velocity;///< Storage for the agent's velocity.
		steer::Vector2                              m_heading;///< Storage for the agent's normalized vector pointing in the direction it is headed.
		steer::Vector2                              m_side;///< A vector perpendicular to the direction the agent is heading.
		float                                       m_mass;///< Mass of the agent.
		float                                       m_timeElapsed;///< The time elapsed since the last frame - useful for some steering behavior calcuations.
		steer::Vector2                              m_offset;///< any offset used for formations or offset pursuit
		float                                       m_viewDistance;///< how far the agent can 'see'
		Uint32                                      m_deceleration;///< Deceleration type (slow, normal, fast) - SEE DECLERATION ENUM in BEHAVIORHELPERS.HPP
		float                                       m_boxLength;///< length of the 'detection box' utilized in obstacle avoidance
		float                                       m_wallDetectionFeelerLength;///< the length of the 'feeler/s' used in wall detection
		float                                       m_maxSpeed;///< The maximum speed at which the agent can travel.
		float                                       m_maxForce;///< The maximum force the agent can use to propel itself.
		float                                       m_maxTurnRate;///< The maximum rate at which the agent can rotate.
		steer::Vector2                              m_target;///< For setting the agent's target.
		float										m_threatRange;///<  Range the object of interest must be in to trigger evasive action.
		float										m_decelerationTweaker;///< Value used to tweak deceleration.
		float										m_distanceBuffer;///< Value used to set the distance buffer.
		steer::Vector2								m_wanderTarget;///< the current position on the wander circle the agent is attempting to steer towards
		float										m_wanderJitter;///< Amount of displacement along the constraining circle for the wandering agent.
		float										m_wanderRadius;///< The radius of the constraining circle for the wandering agent.
		float										m_wanderDistance;///< Distance the wander circle is projected in front of the agent.
	};
} //end steeriously namespace

#endif // AGENT_HPP
