#ifndef SuperComponent_HPP
#define SuperComponent_HPP

#include <steeriously/libinc.hpp>

namespace steer
{
    /**
    *\class SuperComponent
    *\brief An example implementation of the an agent with every steering behavior implemented.
    **/
	class SuperComponent : public steer::Agent
	{
	public:
		SuperComponent(steer::BehaviorParameters* params);
		virtual ~SuperComponent();

		void setParams(steer::BehaviorParameters* params) { m_params = params; };
		steer::BehaviorParameters* getParams() { return m_params; };

		void Update(float dt);

		/**
		* \fn float getRotation();
		* \brief Get the value of the seek component rotation.
		**/
		float getRotation() { return m_rotation; };

		void setNeighbors(std::vector<SuperComponent*>* n) { m_neighbors = n; };
		std::vector<SuperComponent*>* getNeighbors() { return m_neighbors; };

		void setObstacles(std::vector<SphereObstacle*>* o) { m_obstacles = o; };
		std::vector<SphereObstacle*>* getObstacles() { return m_obstacles; };

		void setWalls(std::vector<Wall*>* w) { m_walls = w; };
		std::vector<Wall*>* getWalls() { return m_walls; };

		void setPath(steer::Path* p){m_path = p;};
        steer::Path* getPath() const {return m_path;};

		//pure virtual - must implement see Agent.hpp
		virtual bool on(steer::behaviorType behavior) override { return (m_iFlags & behavior) == behavior; };

		void alignmentOn() { m_iFlags |= steer::behaviorType::alignment; };
		void separationOn() { m_iFlags |= steer::behaviorType::separation; };
		void cohesionOn() { m_iFlags |= steer::behaviorType::cohesion; };
		void seekOn() { m_iFlags |= steer::behaviorType::seek; };
		void wanderOn() { m_iFlags |= steer::behaviorType::wander; };
		void wallAvoidanceOn() { m_iFlags |= steer::behaviorType::wallAvoidance; };
		void obstacleAvoidanceOn() { m_iFlags |= steer::behaviorType::obstacleAvoidance; };
        void pursuitOn(){m_iFlags |= steer::behaviorType::pursuit;};
        void offsetPursuitOn(){m_iFlags |= steer::behaviorType::offsetPursuit;};
        void interposeOn(){m_iFlags |= steer::behaviorType::interpose;};
        void hideOn(){m_iFlags |= steer::behaviorType::hide;};
        void pathFollowingOn(){m_iFlags |= steer::behaviorType::followPath;};
        void fleeOn(){m_iFlags |= steer::behaviorType::flee;};
        void evadeOn(){m_iFlags |= steer::behaviorType::evade;};
        void arriveOn(){m_iFlags |= steer::behaviorType::arrive;};

		void cohesionOff() { if (on(steer::behaviorType::cohesion)) m_iFlags ^= steer::behaviorType::cohesion; };
		void separationOff() { if (on(steer::behaviorType::separation)) m_iFlags ^= steer::behaviorType::separation; };
		void alignmentOff() { if (on(steer::behaviorType::alignment)) m_iFlags ^= steer::behaviorType::alignment; };
		void seekOff() { if (on(steer::behaviorType::seek)) m_iFlags ^= steer::behaviorType::seek; };
		void wanderOff() { if (on(steer::behaviorType::wander)) m_iFlags ^= steer::behaviorType::wander; };
		void wallAvoidanceOff() { if (on(steer::behaviorType::wallAvoidance)) m_iFlags ^= steer::behaviorType::wallAvoidance; };
		void obstacleAvoidanceOff() { if (on(steer::behaviorType::obstacleAvoidance)) m_iFlags ^= steer::behaviorType::obstacleAvoidance; };
        void pursuitOff(){if(on(steer::behaviorType::pursuit))   m_iFlags ^=steer::behaviorType::pursuit;};
        void offsetPursuitOff(){if(on(steer::behaviorType::offsetPursuit))   m_iFlags ^=steer::behaviorType::offsetPursuit;}
        void interposeOff(){if(on(steer::behaviorType::interpose))   m_iFlags ^=steer::behaviorType::interpose;}
        void hideOff(){if(on(steer::behaviorType::hide))   m_iFlags ^=steer::behaviorType::hide;}
        void pathFollowingOff(){if(on(steer::behaviorType::followPath))   m_iFlags ^=steer::behaviorType::followPath;}
        void fleeOff(){if(on(steer::behaviorType::flee))   m_iFlags ^=steer::behaviorType::flee;}
        void evadeOff(){if(on(steer::behaviorType::evade))   m_iFlags ^=steer::behaviorType::evade;}
        void arriveOff(){if(on(steer::behaviorType::arrive))   m_iFlags ^=steer::behaviorType::arrive;}

		bool isCohesionOn() { return on(steer::behaviorType::cohesion); };
		bool isSeparationOn() { return on(steer::behaviorType::separation); };
		bool isAlignmentOn() { return on(steer::behaviorType::alignment); };
		bool isSeekOn() { return on(steer::behaviorType::seek); };
		bool isWanderOn() { return on(steer::behaviorType::wander); };
		bool isWallAvoidanceOn() { return on(steer::behaviorType::wallAvoidance); };
		bool isObstacleAvoidanceOn() { return on(steer::behaviorType::obstacleAvoidance); };
		bool isFlockingOn() { return isCohesionOn() && isAlignmentOn() && isSeparationOn() && isSeekOn() && isWanderOn() && isWallAvoidanceOn() && isObstacleAvoidanceOn(); };
        bool isPursuitOn(){return on(steer::behaviorType::pursuit);};
        bool isOffsetPursuitOn(){return on(steer::behaviorType::offsetPursuit);};
        bool isInterposeOn(){return on(steer::behaviorType::interpose);};
        bool isHideOn(){return on(steer::behaviorType::hide);};
        bool isPathFollowingOn(){return on(steer::behaviorType::followPath);};
        bool isFleeOn(){return on(steer::behaviorType::flee);};
        bool isEvadeOn(){return on(steer::behaviorType::evade);};
        bool isArriveOn(){return on(steer::behaviorType::arrive);};

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

		void setEvadeAgent(steer::Agent* a){m_evadeAgent = a;};
        steer::Agent* getEvadeAgent() const {return m_evadeAgent;};

		void setPursuitAgent(steer::Agent* a){m_pursuitAgent = a;};
        steer::Agent* getPursuitAgent() const {return m_pursuitAgent;};

        void setLeader(steer::Agent* l){m_leader = l;};
        steer::Agent* getLeader() const {return m_leader;};

        void setInterposeAgents(steer::Agent* a, steer::Agent* b){m_interposeAgentA = a; m_interposeAgentB = b;};
        void setInterposeAgentA(steer::Agent* a){m_interposeAgentA = a;};
        void setInterposeAgentB(steer::Agent* b){m_interposeAgentB = b;};

        steer::Agent* getInterposeAgentA() const {return m_interposeAgentA;};
        steer::Agent* getInterposeAgentB() const {return m_interposeAgentB;};

        void setHideAgent(steer::Agent* a){m_hideAgent = a;};
        steer::Agent* getHideAgent() const {return m_hideAgent;};

		//pure virtual - must implement see Agent.hpp
		virtual Vector2 Calculate() override;

		//optional virtual
		virtual Vector2 calculateWeightedSum() override;

	public:
		float											m_weightSeek;///< Multiplier - can be adjusted to effect strength of the seeking behavior.
		float											m_weightFlee;///< Multiplier - can be adjusted to effect strength of the fleeing behavior.
		float											m_weightArrive;///< Multiplier - can be adjusted to effect strength of the arriving behavior.
		float											m_weightPursuit;///< Multiplier - can be adjusted to effect strength of the pursuing behavior.
		float											m_weightEvade;///< Multiplier - can be adjusted to effect strength of the evading behavior.
		float											m_weightInterpose;///< Multiplier - can be adjusted to effect strength of the interposing behavior.
		float											m_weightHide;///< Multiplier - can be adjusted to effect strength of the hiding behavior.
		float											m_weightOffsetPursuit;///< Multiplier - can be adjusted to effect strength of the offset pursuit behavior.
		float                                           m_weightWander;///< Multiplier - can be adjusted to effect strength of the wander behavior.
		float											m_weightAlignment;///< Multiplier - can be adjusted to effect strength of the alignment behavior.
		float											m_weightSeparation;///< Multiplier - can be adjusted to effect strength of the separation behavior.
		float											m_weightCohesion;///< Multiplier - can be adjusted to effect strength of the cohesion behavior.
		float											m_weightObstacleAvoidance;///< Multiplier - can be adjusted to effect strength of the obstacle avoidance behavior.
		float                                           m_weightWallAvoidance;///< Multiplier - can be adjusted to effect strength of the wall avoidance behavior.
		float						                    m_weightPathFollowing;///< Multiplier - can be adjusted to effect strength of the Path Following behavior.
    private:
		Uint32										    m_iFlags;///< binary flags to indicate whether or not a behavior should be active
		float											m_rotation;///< rotation of the component for applying to drawables and other entities.
		steer::Agent*                                   m_evadeAgent;///< The target agent that your entity will be evading.
		steer::Agent*                                   m_pursuitAgent;///< The target agent that your entity will be pursuing.
		steer::Agent*                                   m_leader;///< pointer to agent that is leading the pursuit.
		steer::Agent*                                   m_interposeAgentA;///< pointer to first agent the Interposing agent will get between.
		steer::Agent*                                   m_interposeAgentB;///< pointer to second agent the Interposing agent will get between.
		steer::Agent*                                   m_hideAgent;///< The target agent that your entity will be avoiding.
		std::vector<SuperComponent*>*                   m_neighbors;///< Neighboring flock members used for calculating alignment/separation/cohesion forces.
		std::vector<SphereObstacle*>*					m_obstacles;///< pointer to the obstacles needed to avoid them.
		std::vector<Wall*>*					            m_walls;///< pointer to the walls needed to avoid them.
		steer::Path*                                    m_path;///< pointer to path that the Agent will follow.
		steer::BehaviorParameters*						m_params;///< pointer to flock parameters.
	};
}

#endif // SuperComponent_HPP
