//
// Created by Macx on 2018/12/6.
//

#ifndef SOFTRENDER_MODEL_H
#define SOFTRENDER_MODEL_H

#include <vector>
//#include "../imported/eigen/Eigen/Dense"
#include "Math.hpp"
#include "tgaimage.h"

//using namespace Eigen;
using namespace FW;
class Model {
 private:
  std::vector<Vec3f> verts_;
  std::vector<std::vector<Vec3i> > faces_;
  std::vector<Vec3f> norms_;
  std::vector<Vec2f> uv_;
  TGAImage diffusemap_;
  TGAImage normalmap_;
  TGAImage specularmap_;
  void load_texture(std::string filename, const char *suffix, TGAImage &img);
 public:
  Model(const char *filename);
  ~Model();
  int nverts();
  int nfaces();
  Vec3f normal(int iface, int nthvert);
  Vec3f normal(Vec2f uv);
  Vec3f vert(int i);
  Vec3f vert(int iface, int nthvert);
  Vec2f uv(int iface, int nthvert);
  TGAColor diffuse(Vec2f uv);
  float specular(Vec2f uv);
  std::vector<int> face(int idx);
};
#endif //SOFTRENDER_MODEL_H
