#ifndef FlockingComponent_HPP
#define FlockingComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/SphereObstacle.hpp>
#include <steeriously/Wall.hpp>

namespace steer
{
    /**
    *\class FlockingComponent
    *\brief An example implementation of the flocking steering behavior.
    *       The agent will interact with other members of the flock
    *       utilizing alignment, separation, cohesion, and wandering behaviors.
    **/
	class FlockingComponent : public steer::Agent
	{
	public:
		FlockingComponent(steer::BehaviorParameters* params);
		virtual ~FlockingComponent();

		void setParams(steer::BehaviorParameters* params) { m_params = params; };
		steer::BehaviorParameters* getParams() { return m_params; };

		void Update(float dt);

		/**
		* \fn float getRotation();
		* \brief Get the value of the seek component rotation.
		**/
		float getRotation() { return m_rotation; };

		void setNeighbors(std::vector<FlockingComponent*>* n) { m_neighbors = n; };
		std::vector<FlockingComponent*>* getNeighbors() { return m_neighbors; };

		void setObstacles(std::vector<SphereObstacle*>* o) { m_obstacles = o; };
		std::vector<SphereObstacle*>* getObstacles() { return m_obstacles; };

		void setWalls(std::vector<Wall*>* w) { m_walls = w; };
		std::vector<Wall*>* getWalls() { return m_walls; };

		//pure virtual - must implement see Agent.hpp
		virtual bool on(steer::behaviorType behavior) override { return (m_iFlags & behavior) == behavior; };

		void alignmentOn() { m_iFlags |= steer::behaviorType::alignment; };
		void separationOn() { m_iFlags |= steer::behaviorType::separation; };
		void cohesionOn() { m_iFlags |= steer::behaviorType::cohesion; };
		void seekOn() { m_iFlags |= steer::behaviorType::seek; };
		void wanderOn() { m_iFlags |= steer::behaviorType::wander; };
		void wallAvoidanceOn() { m_iFlags |= steer::behaviorType::wallAvoidance; };
		void obstacleAvoidanceOn() { m_iFlags |= steer::behaviorType::obstacleAvoidance; };

		void cohesionOff() { if (on(steer::behaviorType::cohesion)) m_iFlags ^= steer::behaviorType::cohesion; };
		void separationOff() { if (on(steer::behaviorType::separation)) m_iFlags ^= steer::behaviorType::separation; };
		void alignmentOff() { if (on(steer::behaviorType::alignment)) m_iFlags ^= steer::behaviorType::alignment; };
		void seekOff() { if (on(steer::behaviorType::seek)) m_iFlags ^= steer::behaviorType::seek; };
		void wanderOff() { if (on(steer::behaviorType::wander)) m_iFlags ^= steer::behaviorType::wander; };
		void wallAvoidanceOff() { if (on(steer::behaviorType::wallAvoidance)) m_iFlags ^= steer::behaviorType::wallAvoidance; };
		void obstacleAvoidanceOff() { if (on(steer::behaviorType::obstacleAvoidance)) m_iFlags ^= steer::behaviorType::obstacleAvoidance; };

		bool isCohesionOn() { return on(steer::behaviorType::cohesion); };
		bool isSeparationOn() { return on(steer::behaviorType::separation); };
		bool isAlignmentOn() { return on(steer::behaviorType::alignment); };
		bool isSeekOn() { return on(steer::behaviorType::seek); };
		bool isWanderOn() { return on(steer::behaviorType::wander); };
		bool isWallAvoidanceOn() { return on(steer::behaviorType::wallAvoidance); };
		bool isObstacleAvoidanceOn() { return on(steer::behaviorType::obstacleAvoidance); };
		bool isFlockingOn() { return isCohesionOn() && isAlignmentOn() && isSeparationOn() && isSeekOn() && isWanderOn() && isWallAvoidanceOn() && isObstacleAvoidanceOn(); };

		void flockingOn()
		{
			cohesionOn();
			alignmentOn();
			separationOn();
			seekOn();
            wanderOn();
            wallAvoidanceOn();
            obstacleAvoidanceOn();
		};

		void flockingOff()
		{
			cohesionOff();
			alignmentOff();
			separationOff();
			seekOff();
			wanderOff();
			wallAvoidanceOff();
			obstacleAvoidanceOff();
		}

		bool targetAcquired();

		//pure virtual - must implement see Agent.hpp
		virtual Vector2 Calculate() override;

		//optional virtual
		virtual Vector2 calculateWeightedSum() override;

	private:
		float											m_weightSeek;///< Multiplier - can be adjusted to effect strength of the seeking behavior.
		float                                           m_weightWander;///< Multiplier - can be adjusted to effect strength of the wander behavior.
		float											m_weightAlignment;///< Multiplier - can be adjusted to effect strength of the alignment behavior.
		float											m_weightSeparation;///< Multiplier - can be adjusted to effect strength of the separation behavior.
		float											m_weightCohesion;///< Multiplier - can be adjusted to effect strength of the cohesion behavior.
		float											m_weightObstacleAvoidance;///< Multiplier - can be adjusted to effect strength of the obstacle avoidance behavior.
		float                                           m_weightWallAvoidance;///< Multiplier - can be adjusted to effect strength of the wall avoidance behavior.
		Uint32										    m_iFlags;///< binary flags to indicate whether or not a behavior should be active
		float											m_rotation;///< rotation of the component for applying to drawables and other entities.
		std::vector<FlockingComponent*>*                m_neighbors;///< Neighboring flock members used for calculating alignment/separation/cohesion forces.
		std::vector<SphereObstacle*>*					m_obstacles;///< pointer to the obstacles needed to avoid them.
		std::vector<Wall*>*					            m_walls;///< pointer to the walls needed to avoid them.
		steer::BehaviorParameters*						m_params;///< pointer to flock parameters.
	};
}

#endif // FlockingComponent_HPP
