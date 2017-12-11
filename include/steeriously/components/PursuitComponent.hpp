#ifndef PursuitComponent_HPP
#define PursuitComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class PursuitComponent
    *\brief An example implementation of the pursuit steering behavior.
    **/
    class PursuitComponent : public steer::Agent
    {
        public:
            PursuitComponent(steer::BehaviorParameters* params);
            virtual ~PursuitComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the Pursuit component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightPursuit = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the Pursuit component.
			**/
			float getWeight()const { return m_weightPursuit; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the Pursuit component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the Pursuit component rotation.
			**/
			float getRotation() { return m_rotation; };

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void pursuitOn(){m_iFlags |= steer::behaviorType::pursuit;};

            bool isPursuitOn(){return on(steer::behaviorType::pursuit);};
            void pursuitOff(){if(on(steer::behaviorType::pursuit))   m_iFlags ^=steer::behaviorType::pursuit;};

            void setTargetAgent(steer::Agent* a){m_targetAgent = a;};
            steer::Agent* getTargetAgent() const {return m_targetAgent;};

            bool targetAcquired();

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightPursuit;///< Multiplier - can be adjusted to effect strength of the Pursuiting behavior.
            Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
            steer::Agent*               m_targetAgent;///< The target agent that your entity will be pursuing.
    };
}

#endif // PursuitComponent_HPP
