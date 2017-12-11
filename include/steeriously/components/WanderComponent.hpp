#ifndef WANDERCOMPONENT_HPP
#define WANDERCOMPONENT_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>

namespace steer
{
    /**
    *\class WanderComponent
    *\brief An example implementation of the wander steering behavior.
    **/
	class WanderComponent : public steer::Agent
	{
	public:
		WanderComponent(steer::BehaviorParameters* params);
		virtual ~WanderComponent();

		void Update(float dt);

		/**
		*\fn void setWeight(float weight);
		*\brief Set the value of the weight multiplier for the seek component.
		*\param weight - a plain old float.
		**/
		void setWeight(const float weight) { m_weightWander = weight; };

		/**
		* \fn float getWeight();
		* \brief Get the value of the weight multiplier for the seek component.
		**/
		float getWeight()const { return m_weightWander; };

		/**
        * \fn void setRotation(float r);
        * \brief Set the value of the seek component rotation.
        **/
        void setRotation(float r) { m_rotation = r; };

		/**
		* \fn float getRotation();
		* \brief Get the value of the seek component rotation.
		**/
		float getRotation() { return m_rotation; };

		//pure virtual - must implement see Agent.hpp
		virtual bool on(steer::behaviorType behavior) { return (m_iFlags & behavior) == behavior; };

		void wanderOn() { m_iFlags |= steer::behaviorType::wander; };
		void wanderOff() { if (on(steer::behaviorType::wander)) m_iFlags ^= steer::behaviorType::wander; };
		bool isWanderOn() { return on(steer::behaviorType::wander); };

		//pure virtual - must implement see Agent.hpp
		virtual Vector2 Calculate();

	private:
		float						m_weightWander;///< Multiplier - can be adjusted to effect strength of the seeking behavior.
		Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active
		float						m_rotation;///< rotation of the component for applying to drawables or other entities.
	};
}

#endif //WANDERCOMPONENT_HPP
