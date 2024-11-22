#pragma once

#include <numeric>
#include <spdlog/spdlog.h>

#include "Engine/Scene.h"
#include "Labs/3-Rendering/Ray.h"

namespace VCX::Labs::Rendering {

    constexpr float EPS1 = 1e-2f; // distance to prevent self-intersection
    constexpr float EPS2 = 1e-8f; // angle for parallel judgement
    constexpr float EPS3 = 1e-4f; // relative distance to enlarge kdtree

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord);

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord);

    struct Intersection {
        float t, u, v; // ray parameter t, barycentric coordinates (u, v)
    };

    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3);

    struct RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec4         IntersectAlbedo;   // [Albedo   (vec3), Alpha     (float)]
        glm::vec4         IntersectMetaSpec; // [Specular (vec3), Shininess (float)]
    };

    struct TrivialRayIntersector {
        Engine::Scene const * InternalScene = nullptr;

        TrivialRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
        }

        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    if (! IntersectTriangle(its, ray, p1, p2, p3)) continue;
                    if (its.t < EPS1 || its.t > tmin) continue;
                    tmin = its.t, umin = its.u, vmin = its.v, modelIdx = i, meshIdx = j;
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);

            return result;
        }
    };

    /* Optional: write your own accelerated intersector here */

    struct KDT {
        std::vector < std::pair < int, int > > nodes;
        glm::vec3 pmin, pmax; // surround
        int split_pos;
        KDT * ls;
        KDT * rs;
        KDT()  = default;
    };

    # define NODES_LIM 20

    KDT *build(Engine::Scene const * InternalScene, std::vector < std::pair < int, int > > nodes, int lim, int dep = 0) {
        if(nodes.size() == 0) return nullptr;
        KDT *x = new KDT;
        x->pmin = glm::vec3(1e9, 1e9, 1e9);
        x->pmax = -x->pmin;
        auto check = [&](glm::vec3 p) {
            for(int i = 0; i < 3; i++) {
                x->pmin[i] = std::min(x->pmin[i], p[i]);
                x->pmax[i] = std::max(x->pmax[i], p[i]);
            }
        };
        for(auto p : nodes) {
            auto const & model          = InternalScene->Models[p.first];
            std::uint32_t const *face   = model.Mesh.Indices.data() + p.second;
            glm::vec3 const &     p1    = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2    = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3    = model.Mesh.Positions[face[2]];
            check(p1), check(p2), check(p3);
        }
        if(dep == lim || nodes.size() < NODES_LIM) {
            x->ls = x->rs = nullptr;
            x->nodes = nodes;
            return x;
        }
        x->split_pos = dep % 3;
        float mid = (x->pmin[x->split_pos] + x->pmax[x->split_pos]) / 2;
        std::vector < std::pair < int, int > > lnode, rnode;
        for(auto p : nodes) {
            auto const & model          = InternalScene->Models[p.first];
            std::uint32_t const *face   = model.Mesh.Indices.data() + p.second;
            glm::vec3 const &     p1    = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2    = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3    = model.Mesh.Positions[face[2]];
        }
        return x;
    }
    struct KDIntersector {
        Engine::Scene const * InternalScene = nullptr;
        KDT * root = nullptr;
        KDIntersector() = default;
        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
            int maxmodel = InternalScene->Models.size();
            std::vector < std::pair < int, int > > allnodes; 
            for(int i = 0; i < maxmodel; i++) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    allnodes.emplace_back(i, j);
                }
            }
            root = build(InternalScene, allnodes, 8 + 1.3 * log(allnodes.size()));
        }

        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    if (! IntersectTriangle(its, ray, p1, p2, p3)) continue;
                    if (its.t < EPS1 || its.t > tmin) continue;
                    tmin = its.t, umin = its.u, vmin = its.v, modelIdx = i, meshIdx = j;
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);

            return result;
        }
    };

    using RayIntersector = TrivialRayIntersector;

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);

} // namespace VCX::Labs::Rendering