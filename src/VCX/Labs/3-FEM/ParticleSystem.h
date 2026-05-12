#pragma once
#include <vector>
#include <glm/glm.hpp>

namespace VCX::Labs::FEM {
	class ParticleSystem {
    public:
        std::vector<glm::vec3> x;
        std::vector<glm::vec3> v;
        std::vector<glm::vec3> f; 

        std::vector<float>     mass;
        std::vector<float> inv_m;
        std::vector<bool>  is_fixed;

        int size;

        inline int getID(int i, int j, int k, int ny, int nz) {
            return i * (ny + 1) * (nz + 1) + j * (nz + 1) + k;
        }

        void clearForces(const glm::vec3 & gravity);
        void addForce(int id, const glm::vec3 & f_val);
        void step(float dt, float damp = 0.999f);
        void setParticle(int id, const glm::vec3 & pos, bool fixed);
        void resize(int n);
        void addMass(int id, float m_val);
        void updateMass();
	};
}