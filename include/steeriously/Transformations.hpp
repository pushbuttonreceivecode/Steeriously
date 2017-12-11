#ifndef TRANSFORMATIONS_HPP
#define TRANSFORMATIONS_HPP

#include <vector>

#include <steeriously/Matrix.hpp>
#include <steeriously/Vector2.hpp>
#include <steeriously/VectorMath.hpp>

namespace steer //steeriously namepsace
{

    /**
    *   \fn inline std::vector<steer::Vector2> WorldTransform(std::vector<steer::Vector2> &points,
                                                const steer::Vector2   &pos,
                                                const steer::Vector2   &forward,
                                                const steer::Vector2   &side,
                                                const steer::Vector2   &scale);
    *   \brief Given a std::vector of steer::Vector2, a position, orientation, and scale, this function transforms the std::vector of steer::Vector2 into the object's world space.
    *   \param points - a std::vector of steer::Vector2 floats.
    *   \param pos - a steer::Vector2 of floats.
    *   \param forward - a steer::Vector2 of floats.
    *   \param side - a steer::Vector2 of floats.
    *   \param scale - a steer::Vector2 of floats.
    **/
    inline std::vector<steer::Vector2> WorldTransform(std::vector<steer::Vector2> &points,
                                                const steer::Vector2   &pos,
                                                const steer::Vector2   &forward,
                                                const steer::Vector2   &side,
                                                const steer::Vector2   &scale)
    {
        //copy the original vertices into the buffer about to be transformed
        std::vector<steer::Vector2> TranVector2Ds = points;

        //create a transformation matrix
        steer::Matrix2D matTransform;

        //scale
        if ( (scale.x != 1.0) || (scale.y != 1.0) )
        {
            matTransform.Scale(scale.x, scale.y);
        }

        //rotate
        matTransform.Rotate(forward, side);

        //and translate
        matTransform.Translate(pos.x, pos.y);

        //now transform the object's vertices
        matTransform.TransformVector2Ds(TranVector2Ds);

        return TranVector2Ds;
    }

    /**
    *   \fn std::vector<steer::Vector2> WorldTransform(std::vector<steer::Vector2> &points,
                                                const steer::Vector2   &pos,
                                                const steer::Vector2   &forward,
                                                const steer::Vector2   &side);
    *   \brief Given a std::vector of steer::Vector2, a position, and orientation, this function transforms the std::vector of steer::Vector2 into the object's world space.
    *   \param points - a std::vector of steer::Vector2.
    *   \param pos - a steer::Vector2 of floats.
    *   \param forward - a steer::Vector2 of floats.
    *   \param side - a steer::Vector2 of floats.
    **/
    inline std::vector<steer::Vector2> WorldTransform(std::vector<steer::Vector2> &points,
                                     const steer::Vector2   &pos,
                                     const steer::Vector2   &forward,
                                     const steer::Vector2   &side)
    {
        //copy the original vertices into the buffer about to be transformed
        std::vector<steer::Vector2> TranVector2Ds = points;

        //create a transformation matrix
        steer::Matrix2D matTransform;

        //rotate
        matTransform.Rotate(forward, side);

        //and translate
        matTransform.Translate(pos.x, pos.y);

        //now transform the object's vertices
        matTransform.TransformVector2Ds(TranVector2Ds);

        return TranVector2Ds;
    }

    /**
    *   \fn steer::Vector2 PointToWorldSpace(const steer::Vector2 &point,
                                        const steer::Vector2 &AgentHeading,
                                        const steer::Vector2 &AgentSide,
                                        const steer::Vector2 &AgentPosition);
    *   \brief Transforms a point from the agent's local space into world space.
    *   \param point - a steer::Vector2 of floats.
    *   \param AgentHeading - a steer::Vector2 of floats.
    *   \param AgentSide - a steer::Vector2 of floats.
    *   \param AgentPosition - a steer::Vector2 of floats.
    **/
    inline steer::Vector2 PointToWorldSpace(const steer::Vector2 &point,
                                        const steer::Vector2 &AgentHeading,
                                        const steer::Vector2 &AgentSide,
                                        const steer::Vector2 &AgentPosition)
    {
        //make a copy of the point
        steer::Vector2 TransPoint = point;

        //create a transformation matrix
        steer::Matrix2D matTransform;

        //rotate
        matTransform.Rotate(AgentHeading, AgentSide);

        //and translate
        matTransform.Translate(AgentPosition.x, AgentPosition.y);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransPoint);

        return TransPoint;
    }

    /**
    *   \fn steer::Vector2 VectorToWorldSpace(const steer::Vector2 &vec,
                                         const steer::Vector2 &AgentHeading,
                                         const steer::Vector2 &AgentSide);
    *   \brief Transforms a vector from the agent's local space into world space.
    *   \param vec - a steer::Vector2 of floats.
    *   \param AgentHeading - a steer::Vector2 of floats.
    *   \param AgentSide - a steer::Vector2 of floats.
    **/
    inline steer::Vector2 VectorToWorldSpace(const steer::Vector2 &vec,
                                         const steer::Vector2 &AgentHeading,
                                         const steer::Vector2 &AgentSide)
    {
        //make a copy of the point
        steer::Vector2 TransVec = vec;

        //create a transformation matrix
        steer::Matrix2D matTransform;

        //rotate
        matTransform.Rotate(AgentHeading, AgentSide);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransVec);

        return TransVec;
    }

    /**
    *   \fn steer::Vector2 PointToLocalSpace(const steer::Vector2 &point,
                                 const steer::Vector2 &AgentHeading,
                                 const steer::Vector2 &AgentSide,
                                 const steer::Vector2 &AgentPosition);
    *   \brief Transforms a vector from world space into the agent's local space.
    *   \param point - a steer::Vector2 of floats.
    *   \param AgentHeading - a steer::Vector2 of floats.
    *   \param AgentSide - a steer::Vector2 of floats.
    *   \param AgentPosition - a steer::Vector2 of floats.
    **/
    inline steer::Vector2 PointToLocalSpace(const steer::Vector2 &point,
                                 const steer::Vector2 &AgentHeading,
                                 const steer::Vector2 &AgentSide,
                                 const steer::Vector2 &AgentPosition)
    {

        //make a copy of the point
        steer::Vector2 TransPoint = point;

        //create a transformation matrix
        steer::Matrix2D matTransform;

        float Tx = steer::VectorMath::dotProduct(AgentPosition.GetReverse(), AgentHeading);
        float Ty = steer::VectorMath::dotProduct(AgentPosition.GetReverse(), AgentSide);

        //create the transformation matrix
        matTransform._11(AgentHeading.x); matTransform._12(AgentSide.x);
        matTransform._21(AgentHeading.y); matTransform._22(AgentSide.y);
        matTransform._31(Tx);           matTransform._32(Ty);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransPoint);

        return TransPoint;
    }

    /**
    *   \fn steer::Vector2 VectorToLocalSpace(const steer::Vector2 &vec,
                                 const steer::Vector2 &AgentHeading,
                                 const steer::Vector2 &AgentSide);
    *   \brief Transforms a vector from world space into the agent's local space.
    *   \param vec - a steer::Vector2 of floats.
    *   \param AgentHeading - a steer::Vector2 of floats.
    *   \param AgentSide - a steer::Vector2 of floats.
    **/
    inline steer::Vector2 VectorToLocalSpace(const steer::Vector2 &vec,
                                 const steer::Vector2 &AgentHeading,
                                 const steer::Vector2 &AgentSide)
    {

        //make a copy of the point
        steer::Vector2 TransPoint = vec;

        //create a transformation matrix
        steer::Matrix2D matTransform;

        //create the transformation matrix
        matTransform._11(AgentHeading.x); matTransform._12(AgentSide.x);
        matTransform._21(AgentHeading.y); matTransform._22(AgentSide.y);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransPoint);

        return TransPoint;
    }

    /**
    *   \fn void Vec2DRotateAroundOrigin(steer::Vector2& v, float ang);
    *   \brief Rotates a vector by an angle in radians around the origin.
    *   \param v - a steer::Vector2 of floats.
    *   \param ang - a plain old float.
    **/
    inline void Vec2DRotateAroundOrigin(steer::Vector2& v, float ang)
    {
        //create a transformation matrix
        steer::Matrix2D mat;

        //rotate
        mat.Rotate(ang);

        //now transform the object's vertices
        mat.TransformVector2Ds(v);
    }

    /**
    *   \fn std::vector<steer::Vector2> CreateWhiskers(unsigned int NumWhiskers,
                                                float WhiskerLength,
                                                float fov,
                                                steer::Vector2 facing,
                                                steer::Vector2 origin);
    *   \brief Given an origin, a facing direction, a 'field of view' describing the
                limit of the outer whiskers, a whisker length and the number of whiskers
                this method returns a vector containing the end positions of a series
                of whiskers radiating away from the origin and with equal distance between
                them. (like the spokes of a wheel clipped to a specific segment size);
    *   \param NumWhiskers - a plain old unsigned int.
    *   \param WhiskerLength - a plain old float.
    *   \param facing - a steer::Vector2 of floats.
    *   \param origin - a steer::Vector2 of floats.
    **/
    inline std::vector<steer::Vector2> CreateWhiskers(unsigned int  NumWhiskers,
                                                float               WhiskerLength,
                                                float               fov,
                                                steer::Vector2      facing,
                                                steer::Vector2      origin)
    {
        //this is the magnitude of the angle separating each whisker
        float SectorSize = fov/(float)(NumWhiskers-1);

        std::vector<steer::Vector2> whiskers;
        steer::Vector2 temp;
        float angle = -fov*0.5;

        for (unsigned int w=0; w<NumWhiskers; ++w)
        {
            //create the whisker extending outwards at this angle
            temp = facing;
            Vec2DRotateAroundOrigin(temp, angle);
            whiskers.push_back(origin + WhiskerLength * temp);

            angle+=SectorSize;
        }

        return whiskers;
    }

} //namespace steeriously

#endif //TRANSFORMATIONS_HPP

