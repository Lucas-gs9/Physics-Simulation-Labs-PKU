#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ImageRGB.h"
#include "ParticleSystem.h"


namespace VCX::Labs::FEM {
    struct Ray {
        glm::vec3 origin;
        glm::vec3 direction;
    };

    Ray getMouseRay(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize);
    bool getSelectedId(ImVec2 const & mousePos, Engine::Camera const & camera, std::pair<uint32_t, uint32_t> windowSize, ParticleSystem & ps, int& selectedId);
    void svd(const glm::mat3 & F, glm::mat3 & U, glm::vec3 & sigma, glm::mat3 & V);
    glm::mat3 getInverseFromSVD(const glm::mat3 & U, const glm::vec3 & sigma, const glm::mat3 & V);
}