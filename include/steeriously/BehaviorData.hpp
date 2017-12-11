#ifndef BEHAVIORDATA_HPP
#define BEHAVIORDATA_HPP

#include <steeriously/Utilities.hpp>
#include <steeriously/Vector2.hpp>
#include <steeriously/VectorMath.hpp>

namespace steer
{
	/**
	* \enum behaviorType
	* \brief Bitfield for behavior types. Useful as a representation for combining behaviors.
	**/
	enum behaviorType
	{
		none                = 0x00000,
		seek                = 0x00002,
		flee                = 0x00004,
		arrive              = 0x00008,
		wander              = 0x00010,
		cohesion            = 0x00020,
		separation          = 0x00040,
		alignment           = 0x00080,
		obstacleAvoidance   = 0x00100,
		wallAvoidance       = 0x00200,
		followPath          = 0x00400,
		pursuit             = 0x00800,
		evade               = 0x01000,
		interpose           = 0x02000,
		hide                = 0x04000,
		flock               = 0x08000,
		offsetPursuit       = 0x10000,
	};

	/**
	* \enum Deceleration
	* \brief Useful for applying a scaling factor to deceleration.
	**/
	enum Deceleration
	{
		slow    = 3,
		normal  = 2,
		fast    = 1
	};

	/**
	* \enum summingMethod
	* \brief Useful for indicating what type of summation method to use for steering forces.
	**/
	enum summingMethod
	{
		weightedSum = 1,
		prioritized = 2,
		dithered    = 3
	};

	/**
	* \struct BehaviorParameters
	* \brief Data table with default values used to define variables for guiding steerable objects (agents).
	**/
	struct BehaviorParameters
	{
		steer::Vector2 position                    = steer::Vector2(0.0, 0.0);
		steer::Vector2 velocity                    = steer::Vector2(0.0, 0.0);
		steer::Vector2 heading                     = steer::Vector2(1.0, 1.0);
		steer::Vector2 side                        = steer::VectorMath::perpendicular(steer::Vector2(1.0, 1.0));
		float radius                        = 50.f;
		float mass                          = 1.f;

		unsigned int NumAgents              = 1;
		float neighborhoodRadius            = 100.f;
		float SafeDistance                  = 100.f;
		float ThreatRange                   = 100.f;
		Uint32 deceleration                 = steer::Deceleration::fast;
		float DecelerationTweaker           = 0.3f;
		Uint32 SummingMethod                = steer::summingMethod::weightedSum;

		unsigned int NumObstacles           = 2;
		float MinObstacleRadius             = 10;
		float MaxObstacleRadius             = 30;

		float SteeringForceTweaker          = 200.f;

		float SteeringForce                 = 2.f;
		float MaxSpeed                      = 100.f;
		float MaxForce                      = 400.f;
		float MaxTurnRate                   = 10.f;
		float VehicleMass                   = 1.f;
		float VehicleScale                  = 1.f;

		float SeparationWeight              = 1.f;
		float AlignmentWeight               = 10.f;
		float CohesionWeight                = 10.f;
		float ObstacleAvoidanceWeight       = 10.f;
		float WallAvoidanceWeight           = 20.f;
		float WanderWeight                  = 2.f;
		float SeekWeight                    = 1.f;
		float FleeWeight                    = 1.f;
		float ArriveWeight                  = 1.f;
		float PursuitWeight                 = 1.f;
		float OffsetPursuitWeight           = 1.f;
		float InterposeWeight               = 1.f;
		float HideWeight                    = 1.f;
		float EvadeWeight                   = 0.01f;
		float FollowPathWeight              = 10.f;

		// Originally constants, these were used by the original source to set parameters for the wandering behavior.
		// The radius of the constraining circle for the wander behavior
		float wanderRadius                  = 100.f;
		// Distance the wander circle is projected in front of the agent
		float wanderDistance                = 0.001f;
		// The maximum amount of displacement along the circle each frame
		float wanderJitterPerSecond         = 360.f;
		// Used in path following
		float waypointSeekDistance          = 20.f;
		float waypointSeekDistanceSquared   = waypointSeekDistance*waypointSeekDistance;

		float ViewDistance                  = 100.f;

		float MinDetectionBoxLength         = 40.f;

		float WallDetectionFeelerLength     = 40.f;
	};
}

#endif // BEHAVIORDATA_HPP

