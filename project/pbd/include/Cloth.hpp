#pragma once

#include "Particle.hpp"

#include <atlas/utils/Geometry.hpp>
#include <atlas/gl/Buffer.hpp>
#include <atlas/gl/VertexArrayObject.hpp>

namespace pbd
{

    class Cloth : public atlas::utils::Geometry
    {
    public:
        Cloth();

        void setPosition(atlas::math::Point const& pos);

        void updateGeometry(atlas::core::Time<> const& t) override;
        void renderGeometry(atlas::math::Matrix4 const& projection,
            atlas::math::Matrix4 const& view) override;
        void drawGui() override;

        void resetGeometry() override;

    private:
        float mag(atlas::math::Vector v);
        //atlas::math::Vector normal(int p1, int p2, int p3);
        void constrainDistance(int p1, int p2);

        std::vector<Particle> mParticles;
        std::vector<unsigned int> mIndices;
        atlas::gl::Buffer mVertexBuffer;
        atlas::gl::Buffer mIndexBuffer;
        atlas::gl::VertexArrayObject mVao;

        GLsizei mIndexCount;
        atlas::math::Point mPosition;
        float mMass = 1.0f;
        float mWidth = 10.0f;
        float mLength = 10.0f;
        float mHeight = 10.0f;
        float mG = -9.8f;
        float mRest = 1.0f;
        float mRadius = 2.0f;
    };
}
