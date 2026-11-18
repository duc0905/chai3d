#include "HalfEdge.h"
#include "math/CMaths.h"
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
#include <iostream>
#include <queue>
#include <unordered_map>
#include <utility>

struct PairHash {
  std::size_t
  operator()(const std::pair<unsigned int, unsigned int> &p) const noexcept {
    return (std::size_t(p.first) << 32) |
           p.second; // simple mix if 32-bit into 64-bit
  }
};

namespace MyStuff {

// AI code for de-duplicating the vertices
struct QuantizedPos {
  long long x, y, z;
  bool operator==(const QuantizedPos &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct QuantizedPosHash {
  std::size_t operator()(const QuantizedPos &p) const {
    std::size_t h1 = std::hash<long long>{}(p.x);
    std::size_t h2 = std::hash<long long>{}(p.y);
    std::size_t h3 = std::hash<long long>{}(p.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

void weldDuplicateVertices(chai3d::cMesh &mesh, double tolerance = 1e-6) {
  unsigned int oldNumVerts = mesh.m_vertices->getNumElements();
  unsigned int numTris = mesh.m_triangles->getNumElements();
  if (oldNumVerts == 0 || numTris == 0)
    return;

  double invTol = 1.0 / tolerance;
  std::unordered_map<QuantizedPos, unsigned int, QuantizedPosHash> lookup;
  std::vector<unsigned int> remap(oldNumVerts);

  chai3d::cVertexArrayPtr newVertices = chai3d::cVertexArray::create(
      mesh.m_vertices->getUseNormalData(),
      mesh.m_vertices->getUseTexCoordData(), mesh.m_vertices->getUseColorData(),
      mesh.m_vertices->getUseTangentData(),
      mesh.m_vertices->getUseBitangentData(),
      mesh.m_vertices->getUseUserData());

  // 1. Deduplicate vertices by quantized position
  for (unsigned int i = 0; i < oldNumVerts; ++i) {
    chai3d::cVector3d pos = mesh.m_vertices->getLocalPos(i);
    QuantizedPos key{static_cast<long long>(std::llround(pos.x() * invTol)),
                     static_cast<long long>(std::llround(pos.y() * invTol)),
                     static_cast<long long>(std::llround(pos.z() * invTol))};

    auto it = lookup.find(key);
    if (it != lookup.end()) {
      remap[i] = it->second;
    } else {
      unsigned int newIdx = newVertices->newVertex();
      newVertices->setLocalPos(newIdx, pos);
      if (mesh.m_vertices->getUseNormalData())
        newVertices->setNormal(newIdx, mesh.m_vertices->getNormal(i));
      if (mesh.m_vertices->getUseTexCoordData())
        newVertices->setTexCoord(newIdx, mesh.m_vertices->getTexCoord(i));
      if (mesh.m_vertices->getUseColorData())
        newVertices->setColor(newIdx, mesh.m_vertices->getColor(i));

      lookup[key] = newIdx;
      remap[i] = newIdx;
    }
  }

  // 2. Remap triangle vertex indices
  for (unsigned int t = 0; t < numTris; ++t) {
    unsigned int v0 = remap[mesh.m_triangles->getVertexIndex0(t)];
    unsigned int v1 = remap[mesh.m_triangles->getVertexIndex1(t)];
    unsigned int v2 = remap[mesh.m_triangles->getVertexIndex2(t)];
    mesh.m_triangles->setVertices(t, v0, v1, v2);
  }

  // 3. Swap in the welded vertex array and refresh normals/buffers
  mesh.m_vertices = newVertices;
  mesh.computeAllNormals();
  mesh.markForUpdate(false);
}
// End AI code

BasicHEMesh fromChai3DMesh(chai3d::cMesh &cmesh) {
  weldDuplicateVertices(cmesh);

  BasicHEMesh heMesh;
  std::unordered_map<std::pair<unsigned int, unsigned int>, unsigned int,
                     PairHash>
      edges_map;

  unsigned int nTriangles = cmesh.m_triangles->getNumElements();
  unsigned int nVertices = cmesh.m_vertices->getNumElements();

  heMesh.verts.assign(nVertices, {});
  heMesh.faces.assign(nTriangles, {});
  heMesh.edges.reserve(nTriangles * 3);

  // Initalize the vertices
  for (unsigned int i = 0; i < nVertices; ++i) {
    cVector3d pos = cmesh.m_vertices->getLocalPos(i);
    heMesh.verts[i].pos = pos;
  }

  // Build faces and half edges
  for (unsigned int i = 0; i < nTriangles; i++) {
    unsigned int currNEdges = heMesh.edges.size();

    unsigned int v0 = cmesh.m_triangles->getVertexIndex0(i);
    unsigned int v1 = cmesh.m_triangles->getVertexIndex1(i);
    unsigned int v2 = cmesh.m_triangles->getVertexIndex2(i);

    // e0 = (v0, v1), e1 = (v1, v2), e2 = (v2, v0)
    HE_Edge e0, e1, e2;

    e0.source = v0;
    e1.source = v1;
    e2.source = v2;

    e0.next = currNEdges + 1; // points to e1
    e1.next = currNEdges + 2; // points to e2
    e2.next = currNEdges;     // points to e0

    e0.face = e1.face = e2.face = i;

    heMesh.faces[i].edge = heMesh.edges.size();

    heMesh.edges.push_back(e0);
    heMesh.edges.push_back(e1);
    heMesh.edges.push_back(e2);

    edges_map.insert({{v0, v1}, currNEdges});
    edges_map.insert({{v1, v2}, currNEdges + 1});
    edges_map.insert({{v2, v0}, currNEdges + 2});
  }

  // Link the pairs
  for (auto &he : heMesh.edges) {
    unsigned int currVert = he.source;
    unsigned int theOtherVert = heMesh.edges[he.next].source;

    auto it = edges_map.find({theOtherVert, currVert});

    if (it == edges_map.end()) {
      // This is the mesh border
      // TODO: Handle "edge" case if needed
    } else {
      // Found the pair
      he.pair = it->second;
    }
  }

  return heMesh;
}

PriMoMesh fromHEMesh(BasicHEMesh &heMesh) {
  PriMoMesh mesh;

  mesh.verts = heMesh.verts;
  mesh.edges = heMesh.edges;

  mesh.faces.assign(heMesh.faces.size(), {});

  for (unsigned int i = 0; i < heMesh.faces.size(); i++) {
    mesh.faces[i].edge = heMesh.faces[i].edge;
    mesh.faces[i].translate = {0.0f, 0.0f, 0.0f};
    mesh.faces[i].rotate = {0.0f, 0.0f, 0.0f};
  }

  return mesh;
}

// chai3d::cMesh fromPriMoMesh(const PriMoMesh &primoMesh) {
//   // TODO: Implement
//   chai3d::cMesh mesh;
//
//   for (auto &face : primoMesh.faces) {
//     unsigned int he = face.edge;
//     unsigned int currHe = he;
//     unsigned int idx[3];
//     unsigned int i = 0;
//
//     do {
//       auto &edge = primoMesh.edges[currHe];
//       auto &vert = primoMesh.verts[edge.source];
//
//       idx[i++] = mesh.newVertex(vert.pos);
//
//       currHe = edge.next;
//     } while (currHe != he);
//
//     mesh.newTriangle(idx[0], idx[1], idx[2]);
//   }
//
//   return mesh;
// }

void Primo2Chai3DMesh(const PriMoMesh &primoMesh, chai3d::cMesh &c3Mesh) {
  c3Mesh.clear();

  // size_t nVerts = primoMesh.verts.size();
  // NOTE: temporary for demonstration
  size_t nVerts = primoMesh.faces.size() * 3;
  size_t nTriangles = primoMesh.faces.size();

  c3Mesh.m_vertices->newVertices(nVerts);

  unsigned int vIdx = 0;

  // for (unsigned int i = 0; i < nVerts; i++) {
  //   c3Mesh.m_vertices->setLocalPos(i, primoMesh.verts[i].pos);
  // }

  for (unsigned int i = 0; i < nTriangles; i++) {
    auto &face = primoMesh.faces[i];
    auto *currEdge = &primoMesh.edges[face.edge];
    chai3d::cVector3d centroid;

    for (auto edgeIdx : edges(primoMesh, i)) {
      auto &edge = primoMesh.edges[edgeIdx];
      auto &vert = primoMesh.verts[edge.source];

      centroid += vert.pos;
    }
    centroid /= 3;

    chai3d::cMatrix3d rotMat =
        chai3d::cRotEulerRad(face.rotate(0), face.rotate(1), face.rotate(2),
                             chai3d::cEulerOrder::C_EULER_ORDER_ZYX);

    for (auto edgeIdx : edges(primoMesh, i)) {
      auto &edge = primoMesh.edges[edgeIdx];
      auto &vert = primoMesh.verts[edge.source];

      chai3d::cVector3d p =
          rotMat * (vert.pos - centroid) + centroid + face.translate;

      c3Mesh.m_vertices->setLocalPos(vIdx, p);
      c3Mesh.m_vertices->setColor(vIdx, 0.0f, 0.8f, 0.1f);
      vIdx++;
    }

    c3Mesh.m_triangles->newTriangle(i * 3, i * 3 + 1, i * 3 + 2);
  }

  c3Mesh.computeAllNormals();
}

void naiveOptimize(PriMoMesh &mesh) {
  // Use BFS from the constraint faces
  std::vector<bool> visited(mesh.faces.size(), false);

  std::queue<unsigned int> queue;

  for (unsigned int i = 0; i < mesh.faces.size(); i++) {
    if (mesh.faces[i].isConstrain) {
      queue.push(i);
    }
  }

  while (!queue.empty()) {
    unsigned int faceIdx = queue.front();
    queue.pop();

    if (visited[faceIdx])
      continue;

    auto faces = incidentFaces(mesh, faceIdx);
    auto &face = mesh.faces[faceIdx];

    if (!face.isConstrain) {
      face.translate = {0.0f, 0.0f, 0.0f};
      face.rotate = {0.0f, 0.0f, 0.0f};
    }

    if (faces.size() > 0) {
      for (auto &incidentFace : faces) {
        if (!face.isConstrain) {
          face.translate += mesh.faces[incidentFace].translate;
          face.rotate += mesh.faces[incidentFace].rotate;
        }

        queue.push(incidentFace);
      }

      if (!face.isConstrain) {
        face.translate /= faces.size();
        face.rotate /= faces.size();
      }
    }

    visited[faceIdx] = true;
  }
}

void optimizePriMo(PriMoMesh &mesh) {
  // TODO: Implement
  constexpr unsigned int stepsPerFrame = 100;

  for (unsigned int i = 0; i < stepsPerFrame; i++) {
    naiveOptimize(mesh);
  }
}

} // namespace MyStuff
