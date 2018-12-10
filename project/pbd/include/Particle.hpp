#pragma once

#include <atlas/utils/Geometry.hpp>
#include <atlas/gl/Buffer.hpp>
#include <atlas/gl/VertexArrayObject.hpp>
#include <atlas/gl/Texture.hpp>

namespace pbd
{
    class Particle
    {
    public:
        Particle(float mass, atlas::math::Point position);
        void setMovable(bool b);

        float mMass;
        atlas::math::Point mPosition;
        atlas::math::Point mPrediction;
        atlas::math::Vector mVelocity;
        bool mMovable;
    };
}
