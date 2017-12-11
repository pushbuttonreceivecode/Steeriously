#ifndef WALL_HPP
#define WALL_HPP

#include <steeriously/Vector2.hpp>
#include <steeriously/VectorMath.hpp>

namespace steer
{
    /**
        \class Wall
        \brief A class for constructing 2D walls for wall avoidance behaviors.
    **/
    class Wall
    {
    public:

        /**
        * \fn Wall()
        * \brief Default constructor.
        **/
        Wall();

        /**
        * \fn Wall(bool render, Vector2 A, Vector2 B)
        * \param render - a plain old bool.
        * \param A - a Vector2.
        * \param B - a Vector2.
        * \brief Alternative constructor.
        **/
        Wall(bool render, Vector2 A, Vector2 B);

        /**
        * \fn Wall(bool render, Vector2 A, Vector2 B, Vector2 N)
        * \param render - a plain old bool.
        * \param A - a Vector2.
        * \param B - a Vector2.
        * \param N - a Vector2.
        * \brief Alternative constructor.
        **/
        Wall(bool render, Vector2 A, Vector2 B, Vector2 N);

        virtual ~Wall(){};

        /**
        * \fn bool renderNormal()const
        * \brief Returns a bool indicating if the normal should be rendered.
        **/
        bool        renderNormal()const{return m_renderNormals;}

        /**
        * \fn void toggleRenderNormal()
        * \brief Toggles the bool indicating if the normal should be rendered.
        **/
        void        toggleRenderNormal(){m_renderNormals = !m_renderNormals;}

        /**
        * \fn Vector2 From()const
        * \brief Returns the "from" vector component of the wall.
        **/
        Vector2     From()const  {return m_vA;}

        /**
        * \fn void SetFrom(Vector2 v)
        * \param v - a Vector2.
        * \brief Sets the "from" vector component of the wall.
        **/
        void        SetFrom(Vector2 v);

        /**
        * \fn Vector2 To()const
        * \brief Returns the "to" vector component of the wall.
        **/
        Vector2     To()const    {return m_vB;}

        /**
        * \fn void SetTo(Vector2 v)
        * \param v - a Vector2.
        * \brief Sets the "to" vector component of the wall.
        **/
        void        SetTo(Vector2 v);

        /**
        * \fn Vector2 Normal()const
        * \brief Returns the "normal" vector component of the wall.
        **/
        Vector2     Normal()const{return m_vN;}

        /**
        * \fn void SetNormal(Vector2 n)
        * \param v - a Vector2.
        * \brief Sets the "normal" vector component of the wall.
        **/
        void        SetNormal(Vector2 n);

        /**
        * \fn Vector2 Center()const;
        * \brief Returns the "center" vector component of the wall.
        **/
        Vector2     Center()const{return (m_vA+m_vB)/2.f;}

    private:

        Vector2     m_vA;///< member data for building the wall.
        Vector2     m_vB;///< member data for building the wall.
        Vector2     m_vN;///< member data for building the wall.
        bool        m_renderNormals;///< member data for indicating if the normals should be rendered.

        /**
        * \fn void CalculateNormal()
        * \brief Calculates the wall normal based on the "to" vector and the "normal" vector members.
        **/
        void        CalculateNormal();

    };
}

#endif // WALL_HPP
