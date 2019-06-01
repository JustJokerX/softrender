#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "Model.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

Model::Model(const char *filename) : verts_(), faces_(), norms_(), uv_(), diffusemap_(), normalmap_(), specularmap_() {

  this->m_objReader.ParseFromFile(filename);

  load_texture(filename, "_diffuse.tga", diffusemap_);
  load_texture(filename, "_nm_tangent.tga", normalmap_);
  load_texture(filename, "_spec.tga", specularmap_);

  for (const auto &s : m_objReader.GetShapes()) {
    size_t index_offset = 0;
    for (size_t f = 0; f < s.mesh.num_face_vertices.size(); f++) {
      std::vector<tinyobj::index_t> _face;
      int fv = s.mesh.num_face_vertices[f];
      for (size_t v = 0; v < fv; v++) {
        tinyobj::index_t idx = s.mesh.indices[index_offset + v];
        _face.push_back(idx);
      }
      m_faces.push_back(_face);
      index_offset += fv;
    }
  }

//  for (size_t s = 0; s < m_shapes.size(); s++) {
  // Loop over faces(polygon)
//    size_t index_offset = 0;
//    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
//      int fv = shapes[s].mesh.num_face_vertices[f];

  // Loop over vertices in the face.
//      for (size_t v = 0; v < fv; v++) {
  // access to vertex
//        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

//        tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
//        tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
//        tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];
//        Vec3f vtx(vx, vy, vz);
//        verts_.push_back(vtx);
//
//        tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
//        tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
//        tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
//        Vec3f vn(nx, ny, nz);
//        norms_.push_back(vn);
//
//        tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
//        tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];
//        Vec2f uv(tx, ty);
//        uv_.push_back(uv);
  // Optional: vertex colors
  // tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
  // tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
  // tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];
//      }
//      index_offset += fv;

  // per-face material
//      shapes[s].mesh.material_ids[f];
//    }

//    std::ifstream in;
//    in.open(filename, std::ifstream::in);
//    if (in.fail()) return;
//    std::string line;
//    while (!in.eof()) {
//      std::getline(in, line);
//      std::istringstream iss(line.c_str());
//      char trash;
//    if (!line.compare(0, 2, "v ")) {
//      iss >> trash;
//      Vec3f v;
//      for (int i=0;i<3;i++) iss >> v[i];
//      verts_.push_back(v);
//    }  else if (!line.compare(0, 3, "vn ")) {
//      iss >> trash >> trash;
//      Vec3f n;
//      for (int i=0;i<3;i++) iss >> n[i];
//      norms_.push_back(n);
//    } else if (!line.compare(0, 3, "vt ")) {
//      iss >> trash >> trash;
//      Vec2f uv;
//      for (int i=0;i<2;i++) iss >> uv[i];
//      uv_.push_back(uv);
//    } else
//      if (!line.compare(0, 2, "f ")) {
//        std::vector<Vec3i> f;
//        Vec3i tmp;
//        iss >> trash;
//        while (iss >> tmp[0] >> trash >> tmp[1] >> trash >> tmp[2]) {
//          for (int i = 0; i < 3; i++) tmp[i]--; // in wavefront obj all indices start at 1, not zero
//          f.push_back(tmp);
//        }
//        faces_.push_back(f);
//      }
//    }
//    std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << " vt# " << uv_.size() << " vn# "
//              << norms_.size() << std::endl;

}

Model::~Model() {
}

int Model::nverts() {
  return (int) m_objReader.GetShapes()[0].mesh.indices.size();
}

int Model::nfaces() {
  return (int) m_objReader.GetShapes()[0].mesh.num_face_vertices.size();
}

std::vector<int> Model::face(int idx) {
  std::vector<int> face;
  for (int i = 0; i < (int) faces_[idx].size(); i++) face.push_back(faces_[idx][i][0]);
  return face;
}

Vec3f Model::vert(int i) {
  return verts_[i];
}

Vec3f Model::vert(int iface, int nthvert) {
  return verts_[faces_[iface][nthvert][0]];
}

void Model::load_texture(std::string filename, const char *suffix, TGAImage &img) {
  std::string texfile(filename);
  size_t dot = texfile.find_last_of(".");
  if (dot != std::string::npos) {
    texfile = texfile.substr(0, dot) + std::string(suffix);
    std::cerr << "texture file " << texfile << " loading " << (img.read_tga_file(texfile.c_str()) ? "ok" : "failed")
              << std::endl;
    img.flip_vertically();
  }
}

TGAColor Model::diffuse(Vec2f uvf) {
  Vec2i uv(uvf[0] * diffusemap_.get_width(), uvf[1] * diffusemap_.get_height());
  return diffusemap_.get(uv[0], uv[1]);
}

Vec3f Model::normal(Vec2f uvf) {
  Vec2i uv(uvf[0] * normalmap_.get_width(), uvf[1] * normalmap_.get_height());
  TGAColor c = normalmap_.get(uv[0], uv[1]);
  Vec3f res;
  for (int i = 0; i < 3; i++)
    res[2 - i] = (float) c[i] / 255.f * 2.f - 1.f;
  return res;
}

Vec2f Model::uv(int iface, int nthvert) {
  return uv_[faces_[iface][nthvert][1]];
}

float Model::specular(Vec2f uvf) {
  Vec2i uv(uvf[0] * specularmap_.get_width(), uvf[1] * specularmap_.get_height());
  return specularmap_.get(uv[0], uv[1])[0] / 1.f;
}

Vec3f Model::normal(int iface, int nthvert) {
  int idx = faces_[iface][nthvert][2];
  norms_[idx].normalize();
  return norms_[idx];
}

Vec3f Model::vert(tinyobj::index_t idx) {
  tinyobj::real_t vx = m_objReader.GetAttrib().vertices[3 * idx.vertex_index + 0];
  tinyobj::real_t vy = m_objReader.GetAttrib().vertices[3 * idx.vertex_index + 1];
  tinyobj::real_t vz = m_objReader.GetAttrib().vertices[3 * idx.vertex_index + 2];
  Vec3f vtx(vx, vy, vz);
  return vtx;
}

Vec2f Model::uv(tinyobj::index_t idx) {
  tinyobj::real_t tx = m_objReader.GetAttrib().texcoords[2 * idx.texcoord_index + 0];
  tinyobj::real_t ty = m_objReader.GetAttrib().texcoords[2 * idx.texcoord_index + 1];
  Vec2f uv(tx, ty);
  return uv;
}

Vec3f Model::normal(tinyobj::index_t idx) {
  tinyobj::real_t nx = m_objReader.GetAttrib().normals[3 * idx.normal_index + 0];
  tinyobj::real_t ny = m_objReader.GetAttrib().normals[3 * idx.normal_index + 1];
  tinyobj::real_t nz = m_objReader.GetAttrib().normals[3 * idx.normal_index + 2];
  Vec3f vn(nx, ny, nz);
  vn.normalize();
  return vn;
}
