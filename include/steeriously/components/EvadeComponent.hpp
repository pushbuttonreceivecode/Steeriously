#ifndef EvadeComponent_HPP
#define EvadeComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class EvadeComponent
    *\brief An example implementation of the evasion steering behavior.
    *       The agent will flee from the target while predicting it's trajectory.
    **/
    class EvadeComponent : public steer::Agent
    {
        public:
            EvadeComponent(steer::BehaviorParameters* params);
            virtual ~EvadeComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the Evade component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightEvade = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the Evade component.
			**/
			float getWeight()const { return m_weightEvade; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the Evade component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the Evade component rotation.
			**/
			float getRotation() { return m_rotation; };

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void evadeOn(){m_iFlags |= steer::behaviorType::evade;};

            bool isEvadeOn(){return on(steer::behaviorType::evade);};
            void evadeOff(){if(on(steer::behaviorType::evade))   m_iFlags ^=steer::behaviorType::evade;}

            void setTargetAgent(steer::Agent* a){m_targetAgent = a;};
            steer::Agent* getTargetAgent() const {return m_targetAgent;};

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightEvade;///< Multiplier - can be adjusted to effect strength of the Evading behavior.
            Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
            steer::Agent*               m_targetAgent;///< The target agent that your entity will be pursuing.
    };
}

#endif // EvadeComponent_HPP
