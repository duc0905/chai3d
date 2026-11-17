#include "HalfEdge.h"
#include <queue>
#include <set>
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

BasicHEMesh fromChai3DMesh(chai3d::cMesh &cmesh) {
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

    heMesh.edges.push_back(e0);
    heMesh.edges.push_back(e1);
    heMesh.edges.push_back(e2);

    heMesh.faces[i].edge = v0;

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

chai3d::cMesh fromPriMoMesh(const PriMoMesh &primoMesh) {
  // TODO: Implement
  chai3d::cMesh mesh;

  for (auto &face : primoMesh.faces) {
    unsigned int he = face.edge;
    unsigned int currHe = he;
    unsigned int idx[3];
    unsigned int i = 0;

    do {
      auto &edge = primoMesh.edges[currHe];
      auto &vert = primoMesh.verts[edge.source];

      idx[i++] = mesh.newVertex(vert.pos);

      currHe = edge.next;
    } while (currHe != he);

    mesh.newTriangle(idx[0], idx[1], idx[2]);
  }

  return mesh;
}

void Primo2Chai3DMesh(const PriMoMesh &primoMesh, chai3d::cMesh &c3Mesh) {
  c3Mesh.clear();

  size_t nVerts = primoMesh.verts.size();
  size_t nTriangles = primoMesh.faces.size();

  c3Mesh.m_vertices->newVertices(nVerts);

  for (unsigned int i = 0; i < nVerts; i++) {
    c3Mesh.m_vertices->setLocalPos(i, primoMesh.verts[i].pos);
  }

  for (unsigned int i = 0; i < nTriangles; i++) {
    auto &face = primoMesh.faces[i];
    auto *currEdge = &primoMesh.edges[face.edge];

    unsigned int v0 = currEdge->source;
    currEdge = &primoMesh.edges[currEdge->next];
    unsigned int v1 = currEdge->source;
    currEdge = &primoMesh.edges[currEdge->next];
    unsigned int v2 = currEdge->source;

    c3Mesh.m_triangles->newTriangle(v0, v1, v2);
  }

  c3Mesh.computeAllNormals();
}

void naiveOptimize(PriMoMesh &mesh) {
  // Use BFS from the constraint faces
  std::vector<bool> visited(mesh.faces.size(), false);

  std::queue<unsigned int> queue;

  for (unsigned int i = 0; i < mesh.faces.size(); i++) {
    if (mesh.faces[i].isConstrain) {
      visited[i] = true;
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

    if (faces.size() > 0) {
      for (auto &incidentFace : faces) {
        face.translate += mesh.faces[incidentFace].translate;
        face.rotate += mesh.faces[incidentFace].rotate;

        queue.push(incidentFace);
      }

      face.translate /= faces.size();
      face.rotate /= faces.size();
    }

    visited[faceIdx] = true;
  }
}

void optimizePriMo(PriMoMesh &mesh) {
  // TODO: Implement

  for (unsigned int i = 0; i < 100; i++) {
    naiveOptimize(mesh);
  }
}

} // namespace MyStuff
