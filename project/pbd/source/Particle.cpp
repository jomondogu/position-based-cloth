#include "Particle.hpp"
#include "Paths.hpp"
#include "LayoutLocations.glsl"

#include <atlas/utils/Mesh.hpp>
#include <atlas/core/STB.hpp>
#include <atlas/utils/GUI.hpp>
#include <atlas/math/Coordinates.hpp>
#include <math.h>

namespace pbd
{
    Particle::Particle(float mass, atlas::math::Point position)
    {
        mMass = mass;
        mPosition = position;
        mPrediction = atlas::math::Point(0.0f,0.0f,0.0f);
        mVelocity = atlas::math::Vector(0.0f,0.0f,0.0f);
        mMovable = true;
    }

    void Particle::setMovable(bool b){
        mMovable = b;
    }
}
