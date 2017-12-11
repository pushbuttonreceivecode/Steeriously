#ifndef OffsetPursuitComponent_HPP
#define OffsetPursuitComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class OffsetPursuitComponent
    *\brief An example implementation of the offset pursuit steering behavior.
    **/
    class OffsetPursuitComponent : public steer::Agent
    {
        public:
            OffsetPursuitComponent(steer::BehaviorParameters* params);
            virtual ~OffsetPursuitComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the OffsetPursuit component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightOffsetPursuit = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the OffsetPursuit component.
			**/
			float getWeight()const { return m_weightOffsetPursuit; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the OffsetPursuit component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the OffsetPursuit component rotation.
			**/
			float getRotation() { return m_rotation; };

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void OffsetPursuitOn(){m_iFlags |= steer::behaviorType::offsetPursuit;};

            bool isOffsetPursuitOn(){return on(steer::behaviorType::offsetPursuit);};
            void offsetPursuitOff(){if(on(steer::behaviorType::offsetPursuit))   m_iFlags ^=steer::behaviorType::offsetPursuit;}

            void setLeader(steer::Agent* l){m_leader = l;};

            steer::Agent* getLeader() const {return m_leader;};

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightOffsetPursuit;///< Multiplier - can be adjusted to effect strength of the Offset Pursuit behavior.
            Uint32                      m_iFlags;///< binary flags to indicate whether or not a behavior should be active
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
			steer::Agent*               m_leader;///< pointer to agent that is leading the pursuit.
			steer::BehaviorParameters*	m_params;///< pointer to parameters.
    };
}

#endif // OffsetPursuitComponent_HPP
