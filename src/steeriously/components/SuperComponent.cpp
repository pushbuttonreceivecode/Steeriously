#include <steeriously/components/SuperComponent.hpp>
#include <steeriously/Steeriously.hpp>
#include <steeriously/BehaviorHelpers.hpp>

using namespace steer;

steer::SuperComponent::SuperComponent(steer::BehaviorParameters* params)
	: Agent(params)
	, m_weightSeek(params->SeekWeight)
	, m_weightFlee(params->FleeWeight)
	, m_weightArrive(params->ArriveWeight)
	, m_weightPursuit(params->PursuitWeight)
	, m_weightEvade(params->EvadeWeight)
	, m_weightInterpose(params->InterposeWeight)
	, m_weightOffsetPursuit(params->OffsetPursuitWeight)
	, m_weightHide(params->HideWeight)
	, m_weightWander(params->WanderWeight)
	, m_weightAlignment(params->AlignmentWeight)
	, m_weightSeparation(params->SeparationWeight)
	, m_weightCohesion(params->CohesionWeight)
	, m_weightObstacleAvoidance(params->ObstacleAvoidanceWeight)
	, m_weightWallAvoidance(params->WallAvoidanceWeight)
	, m_iFlags()
	, m_rotation(0.f)
	, m_evadeAgent(nullptr)
	, m_pursuitAgent(nullptr)
	, m_leader(nullptr)
	, m_interposeAgentA(nullptr)
	, m_interposeAgentB(nullptr)
	, m_hideAgent(nullptr)
	, m_neighbors(nullptr)
	, m_obstacles(nullptr)
	, m_walls(nullptr)
	, m_params(params)
{
    arriveOff();
    pursuitOff();
    pathFollowingOff();
    interposeOff();
    evadeOff();
    interposeOff();
    offsetPursuitOff();
    hideOff();
    fleeOff();
    flockingOff();

    //stuff for the wander behavior
	float theta = RandFloat() * TwoPi;

	//create a vector to a target position on the wander circle
	m_wanderTarget = Vector2(m_wanderRadius * cos(theta), m_wanderRadius * sin(theta));
}

steer::SuperComponent::~SuperComponent()
{

}

Vector2 steer::SuperComponent::Calculate()
{
    //reset the steering force
    m_steeringForce = steer::Vector2(0.0, 0.0);

	if(on(steer::behaviorType::alignment) && on(steer::behaviorType::separation) && on(steer::behaviorType::cohesion))
    {
        //tag neighbors...
        TagVehiclesWithinViewRange(this, *m_neighbors, this->m_viewDistance);
    }

    //calculate the force, Luke ;)
	m_steeringForce = calculateWeightedSum();

	return m_steeringForce;
}

Vector2 steer::SuperComponent::calculateWeightedSum()
{
    if(on(steer::behaviorType::evade))
    {
        m_steeringForce = Evade(this, m_evadeAgent) * m_weightEvade;
    }

    if (on(steer::behaviorType::separation))
	{
		m_steeringForce += Separation< SuperComponent*, std::vector<SuperComponent*> >(this, *m_neighbors) * m_weightSeparation;
	}

	if (on(steer::behaviorType::alignment))
	{
		m_steeringForce += Alignment< SuperComponent*, std::vector<SuperComponent*> >(this, *m_neighbors) * m_weightAlignment;
	}

	if (on(steer::behaviorType::cohesion))
	{
		m_steeringForce += Cohesion< SuperComponent*, std::vector<SuperComponent*> >(this, *m_neighbors) * m_weightCohesion;
	}

	if (on(steer::behaviorType::wander))
	{
		m_steeringForce = Wander(this) * m_weightWander;
	}

	if (on(steer::behaviorType::seek))
	{
		m_steeringForce += Seek(this) * m_weightSeek;
	}

    if (on(steer::behaviorType::flee))
    {
        m_steeringForce += Flee(this) * m_weightFlee;
    }

    if (on(steer::behaviorType::arrive))
    {
        m_steeringForce += Arrive(this, this->m_deceleration) * m_weightArrive;
    }

    if (on(steer::behaviorType::pursuit))
    {
        assert(m_pursuitAgent && "pursuit target not assigned");

        m_steeringForce += Pursuit(this, m_pursuitAgent) * m_weightPursuit;
    }

    if (on(steer::behaviorType::offsetPursuit))
    {
        assert (m_leader && "pursuit target not assigned");
        assert ((!m_offset.x == 0.f && !m_offset.y == 0.f) && "No offset assigned");

        m_steeringForce += OffsetPursuit(this, m_leader, *m_params) * m_weightOffsetPursuit;
    }

    if (on(steer::behaviorType::interpose))
    {
        assert (m_interposeAgentA && m_interposeAgentB && "Interpose agents not assigned");

        m_steeringForce += Interpose(this, m_interposeAgentA, m_interposeAgentB, *m_params) * m_weightInterpose;
    }

    if (on(steer::behaviorType::hide))
    {
        assert(m_hideAgent && "Hide target not assigned");

        m_steeringForce += Hide< SuperComponent* >(this, m_hideAgent, *m_obstacles, *m_params) * m_weightHide;
    }

    if (on(steer::behaviorType::followPath))
    {
        m_steeringForce += PathFollowing(this, m_path, *m_params) * m_weightPathFollowing;
    }

    if (on(steer::behaviorType::wallAvoidance))
    {
        m_steeringForce += WallAvoidance< SuperComponent* >(this, *m_walls) * m_weightWallAvoidance;
    }

    if (on(steer::behaviorType::obstacleAvoidance))
    {
        m_steeringForce += ObstacleAvoidance< SuperComponent* >(this, *m_obstacles, *m_params) * m_weightObstacleAvoidance;
    }

	m_steeringForce = steer::VectorMath::truncate(m_steeringForce, getMaxForce());

	return m_steeringForce;
}

bool steer::SuperComponent::targetAcquired()
{
	return getPosition() == getTarget();
}

void steer::SuperComponent::Update(float dt)
{
    //update the time elapsed
    m_timeElapsed += dt;

    //keep a record of its old position so we can update its cell later
    //in this method
    Vector2 OldPosition = getPosition();

    //calculate the combined force from each steering behavior in the
    //vehicle's list
    Vector2 SteeringForce = Calculate();

    //Acceleration = Force/Mass
    Vector2 acceleration = SteeringForce / getMass();

    //update velocity
    m_velocity += acceleration * dt;

    //make sure vehicle does not exceed maximum velocity
    m_velocity = steer::VectorMath::truncate(m_velocity, getMaxSpeed());

    //update the position
    m_agentPosition += m_velocity * dt;
    m_rotation = steer::VectorMath::findAngle(m_velocity);

    //update the heading if the vehicle has a non zero velocity
    if (steer::VectorMath::lengthSquared(m_velocity) > 0.00000001f)
    {
        m_heading = steer::VectorMath::normalize(m_velocity);

        m_side = steer::VectorMath::perpendicular(m_heading);
    }
}
