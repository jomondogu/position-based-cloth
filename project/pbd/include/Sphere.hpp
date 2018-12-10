#pragma once

#include <atlas/utils/Geometry.hpp>
#include <atlas/gl/Buffer.hpp>
#include <atlas/gl/VertexArrayObject.hpp>
#include <atlas/gl/Texture.hpp>

namespace pbd
{

    class Sphere : public atlas::utils::Geometry
    {
    public:
        Sphere(std::string const& textureFile);

        void setPosition(atlas::math::Point const& pos);
        void renderGeometry(atlas::math::Matrix4 const& projection,
            atlas::math::Matrix4 const& view) override;

    private:
        atlas::gl::Buffer mVertexBuffer;
        atlas::gl::Buffer mIndexBuffer;
        atlas::gl::VertexArrayObject mVao;
        atlas::gl::Texture mTexture;

        GLsizei mIndexCount;
        atlas::math::Point mPosition;
        float mRadius = 2.0f;
    };
}
