#ifndef SeekComponent_HPP
#define SeekComponent_HPP

#include <steeriously/Agent.hpp>
#include <steeriously/BehaviorData.hpp>
#include <steeriously/BehaviorHelpers.hpp>
#include <steeriously/Vector2.hpp>

namespace steer
{
    /**
    *\class SeekComponent
    *\brief An example implementation of the seek steering behavior.
    **/
    class SeekComponent : public steer::Agent
    {
        public:
            SeekComponent(steer::BehaviorParameters* params);
            virtual ~SeekComponent();

			/**
			*\fn void setWeight(float weight);
			*\brief Set the value of the weight multiplier for the seek component.
			*\param weight - a plain old float.
			**/
			void setWeight(const float weight) { m_weightSeek = weight; };

			/**
			* \fn float getWeight();
			* \brief Get the value of the weight multiplier for the seek component.
			**/
			float getWeight()const { return m_weightSeek; };

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
            virtual bool on(steer::behaviorType behavior){return (m_iFlags & behavior) == behavior;};

            void seekOn(){m_iFlags |= steer::behaviorType::seek;};

            bool isSeekOn(){return on(steer::behaviorType::seek);};
            void seekOff(){if(on(steer::behaviorType::seek))   m_iFlags ^=steer::behaviorType::seek;}

            bool targetAcquired();

            //pure virtual - must implement see Agent.hpp
            virtual Vector2 Calculate();

            void Update(float dt);

        private:
			float						m_weightSeek;///< Multiplier - can be adjusted to effect strength of the seeking behavior.
            Uint32                      m_iFlags;///<binary flags to indicate whether or not a behavior should be active
			float						m_rotation;///< rotation of the component for applying to drawables or other entities.
    };
}

#endif // SeekComponent_HPP
