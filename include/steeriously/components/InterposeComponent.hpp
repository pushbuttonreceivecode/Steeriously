#ifndef InterposeComponent_HPP
#define InterposeComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class InterposeComponent
    *\brief An example implementation of the interpose steering behavior.
    **/
    class InterposeComponent : public steer::Agent
    {
        public:
            InterposeComponent(steer::BehaviorParameters* params);
            virtual ~InterposeComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the Interpose component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightInterpose = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the Interpose component.
			**/
			float getWeight()const { return m_weightInterpose; };

			/**
			* \fn void setRotation(float r);
			* \brief Set the value of the Interpose component rotation.
			**/
			void setRotation(float r) { m_rotation = r; };

			/**
			* \fn float getRotation();
			* \brief Get the value of the Interpose component rotation.
			**/
			float getRotation() { return m_rotation; };

            //pure virtual - must implement see Agent.hpp
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void InterposeOn(){m_iFlags |= steer::behaviorType::interpose;};

            bool isInterposeOn(){return on(steer::behaviorType::interpose);};
            void InterposeOff(){if(on(steer::behaviorType::interpose))   m_iFlags ^=steer::behaviorType::interpose;}

            void setAgents(steer::Agent* a, steer::Agent* b){m_agentA = a; m_agentB = b;};
            void setAgentA(steer::Agent* a){m_agentA = a;};
            void setAgentB(steer::Agent* b){m_agentB = b;};

            steer::Agent* getAgentA() const {return m_agentA;};
            steer::Agent* getAgentB() const {return m_agentB;};

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightInterpose;///< Multiplier - can be adjusted to effect strength of the Interposeing behavior.
            Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
			steer::Agent*               m_agentA;///< pointer to first agent the Interposing agent will get between.
			steer::Agent*               m_agentB;///< pointer to second agent the Interposing agent will get between.
			steer::BehaviorParameters*	m_params;///< pointer to parameters.
    };
}

#endif // InterposeComponent_HPP
