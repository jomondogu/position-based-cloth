#pragma once

#include "Cloth.hpp"
#include "Sphere.hpp"

#include <atlas/tools/ModellingScene.hpp>
#include <atlas/utils/FPSCounter.hpp>

namespace pbd
{
    class ClothScene : public atlas::tools::ModellingScene
    {
    public:
        ClothScene();

        void updateScene(double time) override;
        void renderScene() override;

    private:
        bool mPlay;
        atlas::utils::FPSCounter mAnimCounter;
        Cloth mCloth;
        Sphere mSphere;
    };
}
