//
// Created by Macx on 2018/12/6.
//

#ifndef SOFTRENDER_MODEL_H
#define SOFTRENDER_MODEL_H

#include <vector>
//#include "../imported/eigen/Eigen/Dense"
#include "Math.hpp"
#include "tgaimage.h"
//#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
//namespace tinyobj{
//class ObjReader;
//}

//using namespace Eigen;
using namespace FW;
class Model {

 public:
  Model(const char *filename);
  ~Model();
  int nverts();
  int nfaces();
  Vec3f normal(int iface, int nthvert);
  Vec3f normal(Vec2f uv);
  Vec3f normal(tinyobj::index_t idx);
  Vec3f vert(int i);
  Vec3f vert(tinyobj::index_t idx);
  Vec3f vert(int iface, int nthvert);
  Vec2f uv(int iface, int nthvert);
  Vec2f uv(tinyobj::index_t idx);
  TGAColor diffuse(Vec2f uv);
  float specular(Vec2f uv);
  std::vector<int> face(int idx);
  std::vector<std::vector<tinyobj::index_t> > m_faces;

 private:
  std::vector<Vec3f> verts_;
  std::vector<Vec3f> m_verts;
  std::vector<std::vector<Vec3i> > faces_;
  std::vector<Vec3f> norms_;
  std::vector<Vec3f> m_norms;
  std::vector<Vec2f> uv_;
  std::vector<Vec2f> m_uv;
  TGAImage diffusemap_;
  TGAImage normalmap_;
  TGAImage specularmap_;
  tinyobj::ObjReader m_objReader;

  void load_texture(std::string filename, const char *suffix, TGAImage &img);
};
#endif //SOFTRENDER_MODEL_H
