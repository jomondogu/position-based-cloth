/*
  TODO:
    - render mesh w/vertices at particle positions
      - if possible in atlas framework?
    - add more constraints (stretch, bend, etc.)
*/

#include "Cloth.hpp"
#include "Paths.hpp"
#include "LayoutLocations.glsl"

#include <atlas/utils/Mesh.hpp>
#include <atlas/core/GLFW.hpp>
#include <atlas/utils/GUI.hpp>
#include <math.h>

namespace pbd
{

    Cloth::Cloth() :
        mVertexBuffer(GL_ARRAY_BUFFER),
        mIndexBuffer(GL_ELEMENT_ARRAY_BUFFER)
    {
        using atlas::utils::Mesh;
        namespace gl = atlas::gl;
        namespace math = atlas::math;

        Mesh sphere;
        std::string path{ DataDirectory };
        path = path + "sphere.obj";
        Mesh::fromFile(path, sphere);

        mIndexCount = static_cast<GLsizei>(sphere.indices().size());

        std::vector<float> data;
        for (std::size_t i = 0; i < sphere.vertices().size(); ++i)
        {
            data.push_back(sphere.vertices()[i].x);
            data.push_back(sphere.vertices()[i].y);
            data.push_back(sphere.vertices()[i].z);

            data.push_back(sphere.normals()[i].x);
            data.push_back(sphere.normals()[i].y);
            data.push_back(sphere.normals()[i].z);
        }

        //create Particle vector grid
        for(int i = 0; i < (int)mWidth; i++){
            for (int j = 0; j < (int)mLength; j++){
                float mass = mMass/(mWidth*mLength);
                atlas::math::Vector pos;
                pos = atlas::math::Vector(mWidth*((float)i/mWidth) - mWidth/2.0f,mHeight,mLength*((float)j/mLength) - mLength/2.0f);
                Particle p(mass, pos);
                mParticles.push_back(p);
            }
        }

        mParticles[0].setMovable(false);
        mParticles[mWidth-1].setMovable(false);
        mParticles[mWidth-1].mPosition += (mParticles[0].mPosition - mParticles[mWidth-1].mPosition)*(1/(2*mWidth));

        mVao.bindVertexArray();
        mVertexBuffer.bindBuffer();
        mVertexBuffer.bufferData(gl::size<float>(data.size()), data.data(),
            GL_STATIC_DRAW);
        mVertexBuffer.vertexAttribPointer(VERTICES_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(0));
        mVertexBuffer.vertexAttribPointer(NORMALS_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(3));

        mVao.enableVertexAttribArray(VERTICES_LAYOUT_LOCATION);
        mVao.enableVertexAttribArray(NORMALS_LAYOUT_LOCATION);

        mIndexBuffer.bindBuffer();
        mIndexBuffer.bufferData(gl::size<GLuint>(sphere.indices().size()),
            sphere.indices().data(), GL_STATIC_DRAW);

        mIndexBuffer.unBindBuffer();
        mVertexBuffer.unBindBuffer();
        mVao.unBindVertexArray();

        std::vector<gl::ShaderUnit> shaders
        {
            {std::string(ShaderDirectory) + "Cloth.vs.glsl", GL_VERTEX_SHADER},
            {std::string(ShaderDirectory) + "Cloth.fs.glsl", GL_FRAGMENT_SHADER}
        };

        mShaders.emplace_back(shaders);
        mShaders[0].setShaderIncludeDir(ShaderDirectory);
        mShaders[0].compileShaders();
        mShaders[0].linkShaders();

        auto var = mShaders[0].getUniformVariable("model");
        mUniforms.insert(UniformKey("model", var));
        var = mShaders[0].getUniformVariable("projection");
        mUniforms.insert(UniformKey("projection", var));
        var = mShaders[0].getUniformVariable("view");
        mUniforms.insert(UniformKey("view", var));

        mShaders[0].disableShaders();
        mModel = math::Matrix4(1.0f);
    }

    void Cloth::setPosition(atlas::math::Point const& pos)
    {
        mPosition = pos;
    }

    void Cloth::updateGeometry(atlas::core::Time<> const& t)
    {
        //for each particle in mesh:
          //particle.velocity = particle.velocity + t*(particle.weight)*(external forces)*(particle.position)
            //Symplectic Euler: vi(t0 + t) = vi(t0) + t(fi/mi)t0
        for (uint i = 0; i < mParticles.size(); i++){
            if (mParticles[i].mMovable){
                mParticles[i].mVelocity += t.deltaTime*atlas::math::Vector(0.0f,mG,0.0f);
            }else{
                mParticles[i].mVelocity = atlas::math::Vector(0.0f,0.0f,0.0f);
            }
        }

        //for each particle in mesh:
          //particle.posprediction = particle.position + t*particle.velocity
            //Symplectic Euler: xi(t0 + t) = xi(t0) + t(vi(t0 + t))
        for (uint i = 0; i < mParticles.size(); i++){
            mParticles[i].mPrediction = mParticles[i].mPosition + t.deltaTime*mParticles[i].mVelocity;
        }

        //iteratively:
          //project constraints onto each particle.posprediction
        int iterations = 100;
        for (int iter = 0; iter < iterations; iter++){
            for(int i = 0; i < (int)mWidth; i++){
                for(int j = 0; j < (int)mLength; j++){
                    int index = j*(int)mWidth+i;
                    if(i > 0){
                        constrainDistance(index, index-1);
                    }
                    if(i < mWidth-1){
                        constrainDistance(index, index+1);
                    }
                    if(j > 0){
                        constrainDistance(index, index-(int)mWidth);
                    }
                    if(j < mHeight-1){
                        constrainDistance(index, index+(int)mWidth);
                    }
                }
            }

            //for each particle in mesh:
              //update particle.posprediction based on collision constraints
            for (uint i = 0; i < mParticles.size(); i++){
                //collision constraint with sphere of radius 2 at position
                atlas::math::Vector outvector = mParticles[i].mPrediction - mPosition;
                if (mag(outvector) < mRadius){
                    mParticles[i].mPrediction += normalize(outvector)*(mRadius-mag(outvector));
                }

                //collision with "ground" at y = 0
                if (mParticles[i].mPrediction.y < 0.0f){
                    mParticles[i].mPrediction.y = 0.0f;
                }
            }
        }

        //for each particle in mesh:
          //particle.velocity = (particle.posprediction - particle.position)/t
          //particle.position = particle.posprediction
        for (uint i = 0; i < mParticles.size(); i++){
            if(mParticles[i].mMovable){
                mParticles[i].mVelocity = (mParticles[i].mPrediction - mParticles[i].mPosition)/t.deltaTime;
                mParticles[i].mPosition = mParticles[i].mPrediction;
            }
        }

    }

    void Cloth::renderGeometry(atlas::math::Matrix4 const& projection,
        atlas::math::Matrix4 const& view)
    {
        //currently renders small spheres at particle positions
        namespace math = atlas::math;

        mShaders[0].hotReloadShaders();
        if (!mShaders[0].shaderProgramValid())
        {
            return;
        }

        mShaders[0].enableShaders();

        mVao.bindVertexArray();
        mIndexBuffer.bindBuffer();

        for (uint i = 0; i < mParticles.size(); i++){
            auto mModeli = glm::translate(mModel, mParticles[i].mPosition) * glm::scale(atlas::math::Matrix4(1.0f), atlas::math::Vector(0.1f));
            glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &mModeli[0][0]);
            glUniformMatrix4fv(mUniforms["projection"], 1, GL_FALSE,
                &projection[0][0]);
            glUniformMatrix4fv(mUniforms["view"], 1, GL_FALSE, &view[0][0]);
            const math::Vector grey{ 0.5f, 0.5f, 0.5f };
            glUniform3fv(mUniforms["materialColour"], 1, &grey[0]);

            glDrawElements(GL_TRIANGLES, mIndexCount, GL_UNSIGNED_INT, 0);
        }

        mIndexBuffer.unBindBuffer();
        mVao.unBindVertexArray();

        mShaders[0].disableShaders();
    }

    void Cloth::drawGui(){
        ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Cloth Controls");
        ImGui::End();
    }

    void Cloth::resetGeometry()
    {
        mParticles.clear();
        for(int i = 0; i < (int)mWidth; i++){
            for (int j = 0; j < (int)mLength; j++){
                float mass = mMass/(mWidth*mLength);
                atlas::math::Vector pos;
                pos = atlas::math::Vector(mWidth*((float)i/mWidth) - mWidth/2.0f,mHeight,mLength*((float)j/mLength) - mLength/2.0f);
                Particle p(mass, pos);
                mParticles.push_back(p);
            }
        }
    }

    float Cloth::mag(atlas::math::Vector v)
    {
        return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    }

    void Cloth::constrainDistance(int p1, int p2)
    {
        atlas::math::Vector line = mParticles[p2].mPrediction - mParticles[p1].mPrediction;
        float distance = mag(line);
        atlas::math::Vector correction = line*(1 - mRest/distance);
        if(mParticles[p1].mMovable && mParticles[p1].mMovable){
            correction = 0.5f * correction;
            mParticles[p1].mPrediction += correction;
            mParticles[p2].mPrediction -= correction;
        }else if(!mParticles[p1].mMovable){
            mParticles[p2].mPrediction -= correction;
        }else{
            mParticles[p1].mPrediction += correction;
        }
    }
}
/*

#include "Cloth.hpp"
#include "Paths.hpp"
#include "LayoutLocations.glsl"

#include <atlas/utils/Mesh.hpp>
#include <atlas/core/GLFW.hpp>
#include <atlas/utils/GUI.hpp>
#include <math.h>

namespace pbd
{

    Cloth::Cloth() :
        mVertexBuffer(GL_ARRAY_BUFFER),
        mIndexBuffer(GL_ELEMENT_ARRAY_BUFFER)
    {
        using atlas::utils::Mesh;
        namespace gl = atlas::gl;
        namespace math = atlas::math;

        Mesh sphere;
        std::string path{ DataDirectory };
        path = path + "sphere.obj";
        Mesh::fromFile(path, sphere);

        mIndexCount = static_cast<GLsizei>(sphere.indices().size());

        //create Particle vector grid
        for(int i = 0; i < (int)mLength; i++){
            for (int j = 0; j < (int)mWidth; j++){
                float mass = mMass/(mWidth*mLength);
                atlas::math::Vector pos;
                pos = atlas::math::Vector(mWidth*((float)i/mWidth) - mWidth/2.0f,mHeight,mLength*((float)j/mLength) - mLength/2.0f);
                Particle p(mass, pos);
                mParticles.push_back(p);
            }
        }

        mParticles[0].setMovable(false);
        mParticles[mWidth-1].setMovable(false);
        mParticles[mWidth-1].mPosition += (mParticles[0].mPosition - mParticles[mWidth-1].mPosition)*(1/(2*mWidth));

        for(int i = 0; i < (int)mLength - 1; i++){
            for(int j = 0; j < (int)mWidth - 1; j++){
                mIndices.push_back(j);
                mIndices.push_back(j + i);
                mIndices.push_back(j + 1);
                mIndices.push_back(j + i + 1);
            }
        }

        mIndexCount = static_cast<GLsizei>(mIndices.size());

        std::vector<float> data;

        for (std::size_t i = 0; i < mParticles.size(); ++i)
        {
            data.push_back(mParticles[i].mPosition.x);
            data.push_back(mParticles[i].mPosition.y);
            data.push_back(mParticles[i].mPosition.z);

            atlas::math::Vector norm;
            if(i % ((int)mWidth-1) == 0 && i % ((int)mLength-1) == 0){
                norm = normal(i, i-1, i-(int)mLength);
            }else if(i % ((int)mWidth-1) == 0){
                norm = normal(i, i+(int)mLength, i+(int)mLength);
            }else if(i % ((int)mLength-1) == 0){
                norm = normal(i, i-(int)mLength+1, i+1);
            }else{
                norm = normal(i, i+1, i+(int)mLength);
            }
            data.push_back(norm.x);
            data.push_back(norm.y);
            data.push_back(norm.z);
        }

        mVao.bindVertexArray();
        mVertexBuffer.bindBuffer();
        mVertexBuffer.bufferData(gl::size<float>(data.size()), data.data(),
            GL_STATIC_DRAW);
        mVertexBuffer.vertexAttribPointer(VERTICES_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(0));
        mVertexBuffer.vertexAttribPointer(NORMALS_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(3));

        mVao.enableVertexAttribArray(VERTICES_LAYOUT_LOCATION);
        mVao.enableVertexAttribArray(NORMALS_LAYOUT_LOCATION);

        mIndexBuffer.bindBuffer();
        mIndexBuffer.bufferData(gl::size<GLuint>(sphere.indices().size()),
            mIndices.data(), GL_STATIC_DRAW);

        mIndexBuffer.unBindBuffer();
        mVertexBuffer.unBindBuffer();
        mVao.unBindVertexArray();

        std::vector<gl::ShaderUnit> shaders
        {
            {std::string(ShaderDirectory) + "Cloth.vs.glsl", GL_VERTEX_SHADER},
            {std::string(ShaderDirectory) + "Cloth.fs.glsl", GL_FRAGMENT_SHADER}
        };

        mShaders.emplace_back(shaders);
        mShaders[0].setShaderIncludeDir(ShaderDirectory);
        mShaders[0].compileShaders();
        mShaders[0].linkShaders();

        auto var = mShaders[0].getUniformVariable("model");
        mUniforms.insert(UniformKey("model", var));
        var = mShaders[0].getUniformVariable("projection");
        mUniforms.insert(UniformKey("projection", var));
        var = mShaders[0].getUniformVariable("view");
        mUniforms.insert(UniformKey("view", var));

        mShaders[0].disableShaders();
        mModel = math::Matrix4(1.0f);
    }

    void Cloth::setPosition(atlas::math::Point const& pos)
    {
        mPosition = pos;
    }

    void Cloth::updateGeometry(atlas::core::Time<> const& t)
    {
        //for each particle in mesh:
          //particle.velocity = particle.velocity + t*(particle.weight)*(external forces)*(particle.position)
            //Symplectic Euler: vi(t0 + t) = vi(t0) + t(fi/mi)t0
        for (uint i = 0; i < mParticles.size(); i++){
            if (mParticles[i].mMovable){
                mParticles[i].mVelocity += t.deltaTime*atlas::math::Vector(0.0f,mG,0.0f);
            }else{
                mParticles[i].mVelocity = atlas::math::Vector(0.0f,0.0f,0.0f);
            }
        }

        //for each particle in mesh:
          //particle.posprediction = particle.position + t*particle.velocity
            //Symplectic Euler: xi(t0 + t) = xi(t0) + t(vi(t0 + t))
        for (uint i = 0; i < mParticles.size(); i++){
            mParticles[i].mPrediction = mParticles[i].mPosition + t.deltaTime*mParticles[i].mVelocity;
        }

        //iteratively:
          //project constraints onto each particle.posprediction
        int iterations = 100;
        for (int iter = 0; iter < iterations; iter++){
            for(int i = 0; i < (int)mLength; i++){
                for(int j = 0; j < (int)mWidth; j++){
                    int index = j*(int)mWidth+i;
                    if(i > 0){
                        constrainDistance(index, index-1);
                    }
                    if(i < mWidth-1){
                        constrainDistance(index, index+1);
                    }
                    if(j > 0){
                        constrainDistance(index, index-(int)mWidth);
                    }
                    if(j < mHeight-1){
                        constrainDistance(index, index+(int)mWidth);
                    }
                }
            }

            //for each particle in mesh:
              //update particle.posprediction based on collision constraints
            for (uint i = 0; i < mParticles.size(); i++){
                //collision constraint with sphere of radius 2 at position
                atlas::math::Vector outvector = mParticles[i].mPrediction - mPosition;
                if (mag(outvector) < mRadius){
                    mParticles[i].mPrediction += normalize(outvector)*(mRadius-mag(outvector));
                }

                //collision with "ground" at y = 0
                if (mParticles[i].mPrediction.y < 0.0f){
                    mParticles[i].mPrediction.y = 0.0f;
                }
            }
        }

        //for each particle in mesh:
          //particle.velocity = (particle.posprediction - particle.position)/t
          //particle.position = particle.posprediction
        for (uint i = 0; i < mParticles.size(); i++){
            if(mParticles[i].mMovable){
                mParticles[i].mVelocity = (mParticles[i].mPrediction - mParticles[i].mPosition)/t.deltaTime;
                mParticles[i].mPosition = mParticles[i].mPrediction;
            }
        }

    }

    void Cloth::renderGeometry(atlas::math::Matrix4 const& projection,
        atlas::math::Matrix4 const& view)
    {
        //currently renders small spheres at particle positions
        namespace math = atlas::math;
        namespace gl = atlas::gl;

        mShaders[0].hotReloadShaders();
        if (!mShaders[0].shaderProgramValid())
        {
            return;
        }

        mShaders[0].enableShaders();

        mVao.bindVertexArray();
        mIndexBuffer.bindBuffer();

        std::vector<float> data;

        for (std::size_t i = 0; i < mParticles.size(); ++i)
        {
            data.push_back(mParticles[i].mPosition.x);
            data.push_back(mParticles[i].mPosition.y);
            data.push_back(mParticles[i].mPosition.z);

            atlas::math::Vector norm;
            if(i % ((int)mWidth-1) == 0 && i % ((int)mLength-1) == 0){
                norm = normal(i, i-1, i-(int)mLength);
            }else if(i % ((int)mWidth-1) == 0){
                norm = normal(i, i+(int)mLength, i+(int)mLength);
            }else if(i % ((int)mLength-1) == 0){
                norm = normal(i, i-(int)mLength+1, i+1);
            }else{
                norm = normal(i, i+1, i+(int)mLength);
            }
            data.push_back(norm.x);
            data.push_back(norm.y);
            data.push_back(norm.z);
        }

        mVao.bindVertexArray();
        mVertexBuffer.bindBuffer();
        mVertexBuffer.bufferData(gl::size<float>(data.size()), data.data(),
            GL_STATIC_DRAW);
        mVertexBuffer.vertexAttribPointer(VERTICES_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(0));
        mVertexBuffer.vertexAttribPointer(NORMALS_LAYOUT_LOCATION, 3, GL_FLOAT,
            GL_FALSE, gl::stride<float>(8), gl::bufferOffset<float>(3));

        mVao.enableVertexAttribArray(VERTICES_LAYOUT_LOCATION);
        mVao.enableVertexAttribArray(NORMALS_LAYOUT_LOCATION);

        mIndexBuffer.bindBuffer();
        mIndexBuffer.bufferData(gl::size<GLuint>(mIndices.size()),
            mIndices.data(), GL_STATIC_DRAW);

        glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &mModel[0][0]);
        glUniformMatrix4fv(mUniforms["projection"], 1, GL_FALSE,
            &projection[0][0]);
        glUniformMatrix4fv(mUniforms["view"], 1, GL_FALSE, &view[0][0]);
        const math::Vector grey{ 0.5f, 0.5f, 0.5f };
        glUniform3fv(mUniforms["materialColour"], 1, &grey[0]);

        glDrawElements(GL_LINES, mIndexCount, GL_UNSIGNED_INT, 0);

        glUniformMatrix4fv(mUniforms["model"], 1, GL_FALSE, &mModel[0][0]);
        glUniformMatrix4fv(mUniforms["projection"], 1, GL_FALSE,
            &projection[0][0]);
        glUniformMatrix4fv(mUniforms["view"], 1, GL_FALSE, &view[0][0]);
        const math::Vector grey{ 0.5f, 0.5f, 0.5f };
        glUniform3fv(mUniforms["materialColour"], 1, &grey[0]);

        glDrawElements(GL_LINES, mIndexCount, GL_UNSIGNED_INT, 0);

        mIndexBuffer.unBindBuffer();
        mVertexBuffer.unBindBuffer();
        mVao.unBindVertexArray();

        mShaders[0].disableShaders();
    }

    void Cloth::drawGui(){
        ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Cloth Controls");
        ImGui::End();
    }

    void Cloth::resetGeometry()
    {
        mParticles.clear();
        for(int i = 0; i < (int)mWidth; i++){
            for (int j = 0; j < (int)mLength; j++){
                float mass = mMass/(mWidth*mLength);
                atlas::math::Vector pos;
                pos = atlas::math::Vector(mWidth*((float)i/mWidth) - mWidth/2.0f,mHeight,mLength*((float)j/mLength) - mLength/2.0f);
                Particle p(mass, pos);
                mParticles.push_back(p);
            }
        }
    }

    float Cloth::mag(atlas::math::Vector v)
    {
        return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    }

    atlas::math::Vector Cloth::normal(int p1, int p2, int p3){
        atlas::math::Point pos1 = mParticles[p1].mPosition;
        atlas::math::Point pos2 = mParticles[p2].mPosition;
        atlas::math::Point pos3 = mParticles[p3].mPosition;

        atlas::math::Vector v1 = pos2-pos1;
        atlas::math::Vector v2 = pos3-pos1;

        return atlas::math::Vector(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
    }

    void Cloth::constrainDistance(int p1, int p2)
    {
        atlas::math::Vector line = mParticles[p2].mPrediction - mParticles[p1].mPrediction;
        float distance = mag(line);
        atlas::math::Vector correction = line*(1 - mRest/distance);
        if(mParticles[p1].mMovable && mParticles[p1].mMovable){
            correction = 0.5f * correction;
            mParticles[p1].mPrediction += correction;
            mParticles[p2].mPrediction -= correction;
        }else if(!mParticles[p1].mMovable){
            mParticles[p2].mPrediction -= correction;
        }else{
            mParticles[p1].mPrediction += correction;
        }
    }
}
*/
