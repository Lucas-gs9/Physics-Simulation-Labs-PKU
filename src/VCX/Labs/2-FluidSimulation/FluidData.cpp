#include "FluidData.h"
#include "utils.h"
namespace VCX::Labs::Fluid {
    void Particles::resize(int n) {
        positions.resize(n);
        velocities.resize(n);
        colors.resize(n, glm::vec3(1.f, 1.f, 1.f));
    }

    void Particles::clear() {
        positions.clear();
        velocities.clear();
        colors.clear();
    }

    int Particles::size() const {
        return positions.size();
    }

    glm::vec3 Grid::worldToGrid(const glm::vec3& worldPos) const {
        return worldPos * inv_h;
    }

    glm::ivec3 Grid::getCellCoord(const glm::vec3& worldPos) const {
        glm::vec3 gPos = worldToGrid(worldPos);
        return glm::ivec3(
            std::clamp(int(std::floor(gPos.x)), 0, nx - 1),
            std::clamp(int(std::floor(gPos.y)), 0, ny - 1),
            std::clamp(int(std::floor(gPos.z)), 0, nz - 1));
    }

    int Grid::getCellIdx(const glm::vec3& worldPos) const{
        glm::ivec3 coord = getCellCoord(worldPos);
        return cIdx(coord.x, coord.y, coord.z);
    }

    glm::vec3 Grid::sampleVelocity(const glm::vec3& worldPos) const {
        glm::vec3 g = worldToGrid(worldPos);
        float     uVal = triInterpolate(u, nx + 1, ny, nz, { g.x, g.y - 0.5f, g.z - 0.5f }, [this](int i, int j, int k) { return uIdx(i, j, k); });
        float     vVal = triInterpolate(v, nx, ny + 1, nz, { g.x - 0.5f, g.y, g.z - 0.5f }, [this](int i, int j, int k) { return vIdx(i, j, k); });
        float     wVal = triInterpolate(w, nx, ny, nz + 1, { g.x - 0.5f, g.y - 0.5f, g.z }, [this](int i, int j, int k) { return wIdx(i, j, k); });
    }

    int Grid::size() const{
        return nx * ny * nz;
    }

    void Grid::resetStep() {
        std::fill(u.begin(), u.end(), 0.f);
        std::fill(v.begin(), v.end(), 0.f);
        std::fill(w.begin(), w.end(), 0.f);
        std::fill(density.begin(), density.end(), 0.f);
    }

    void Grid::reset(int resX, int resY, int resZ, float spacing) {
        nx    = resX;
        ny    = resY;
        nz    = resZ;
        h     = spacing;
        inv_h = 1.f / spacing;
        int N = nx * ny * nz;

        u.assign(N, 0.f);
        v.assign(N, 0.f);
        w.assign(N, 0.f);
        u_prev.assign(N, 0.f);
        v_prev.assign(N, 0.f);
        w_prev.assign(N, 0.f);
        pressure.assign(N, 0.f);
        s_field.assign(N, 1.f);
        type.assign(N, CellType::Empty);
        density.assign(N, 0.f);
    }

    void SpatialHash::build(const Grid& grid, const Particles& particles) {
        int numParticles = particles.size();
        int numCells     = grid.size();
        particleList.resize(numParticles);
        cellStart.assign(numCells + 1, 0);

        for (auto & pos : particles.positions) {
            int cIdx = grid.getCellIdx(pos);
            cellStart[cIdx]++;
        }

        int currentOffset = 0;
        for (int i = 0; i < numCells; ++i) {
            int count    = cellStart[i];
            cellStart[i] = currentOffset;
            currentOffset += count;
        }
        cellStart[numCells] = currentOffset;

        std::vector<int> tempCounter(numCells, 0);
        for (int pIdx = 0; pIdx < numParticles; ++pIdx) {
            int cIdx              = grid.getCellIdx(particles.positions[pIdx]);
            int listPos           = cellStart[cIdx] + tempCounter[cIdx];
            particleList[listPos] = pIdx;
            tempCounter[cIdx]++;
        }
    }
}