#pragma once

#include "graphics/CPrimitives.h"
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
#include <concepts>
#include <vector>

using chai3d::cVector3d;

namespace MyStuff {

struct HE_Vertex {
  unsigned int edge; // ptr to an Edge

  cVector3d pos;
};

struct HE_Edge {
  unsigned int source; // ptr to a Vertex

  unsigned int next; // ptr to an Edge

  unsigned int pair; // ptr to an Edge

  unsigned int face; // ptr to a Face
};

struct HE_Face {
  unsigned int edge; // ptr to an Edge
};

template <typename VertType, typename EdgeType, typename FaceType>
struct HE_Mesh {
  std::vector<VertType> verts;

  std::vector<EdgeType> edges;

  std::vector<FaceType> faces;
};

typedef HE_Mesh<HE_Vertex, HE_Edge, HE_Face> BasicHEMesh;

BasicHEMesh fromChai3DMesh(chai3d::cMesh &cmesh);

template <typename VertType = HE_Vertex, std::derived_from<HE_Edge> EdgeType,
          std::derived_from<HE_Face> FaceType>
std::array<unsigned int, 3>
edges(const HE_Mesh<VertType, EdgeType, FaceType> &heMesh,
      unsigned int faceId) {
  std::array<unsigned int, 3> ret;
  auto &face = heMesh.faces[faceId];

  auto *currEdge = &heMesh.edges[face.edge];
  ret[0] = face.edge;
  ret[1] = currEdge->next;
  currEdge = &heMesh.edges[currEdge->next];
  ret[2] = currEdge->next;

  return ret;
}

template <std::derived_from<HE_Vertex> VertType,
          std::derived_from<HE_Edge> EdgeType,
          std::derived_from<HE_Face> FaceType>
std::vector<unsigned int>
neighbors(const HE_Mesh<VertType, EdgeType, FaceType> &heMesh,
          unsigned int vertId) {
  std::vector<unsigned int> ret;
  auto &vert = heMesh.verts[vertId];

  auto e0 = vert.edge;
  unsigned int e = e0;

  do {
    auto *edge = &heMesh.edges[e];          // edge points from vertId.
    auto *pair = &heMesh.edges[edge->pair]; // pair points towards vertId,
                                            // pair's source is a neighbor

    ret.push_back(pair->source);

    e = pair->next;
  } while (e != e0);

  return ret;
}

template <typename VertType = HE_Vertex, std::derived_from<HE_Edge> EdgeType,
          std::derived_from<HE_Face> FaceType>
std::array<unsigned int, 3>
incidentFaces(const HE_Mesh<VertType, EdgeType, FaceType> &heMesh,
              unsigned int faceId) {
  std::array<unsigned int, 3> ret;
  auto &face = heMesh.faces[faceId];

  auto *currEdge = &heMesh.edges[face.edge];
  auto *pairEdge = &heMesh.edges[currEdge->pair];
  ret[0] = pairEdge->face;

  currEdge = &heMesh.edges[currEdge->next];
  pairEdge = &heMesh.edges[currEdge->pair];
  ret[1] = pairEdge->face;

  currEdge = &heMesh.edges[currEdge->next];
  pairEdge = &heMesh.edges[currEdge->pair];
  ret[2] = pairEdge->face;

  return ret;
}

struct PriMo_Face : public HE_Face {
  bool isConstrain = false;
  cVector3d translate;
  cVector3d rotate;
};

// NOTE: Update vertex type to include extrude and intrude amount
typedef HE_Mesh<HE_Vertex, HE_Edge, PriMo_Face> PriMoMesh;

/**
 * Transform vanilla Half edge mesh to PriMoMesh.
 *
 * Add necessary edges, vertices, and faces for PriMo.
 */
PriMoMesh fromHEMesh(BasicHEMesh &heMesh);

chai3d::cMesh fromPriMoMesh(const PriMoMesh &primoMesh);

void Primo2Chai3DMesh(const PriMoMesh &primoMesh, chai3d::cMesh &c3Mesh);

void optimizePriMo(PriMoMesh &mesh);

}; // namespace MyStuff
