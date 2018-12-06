//
// Created by Macx on 2018/12/6.
//

#ifndef SOFTRENDER_MODEL_H
#define SOFTRENDER_MODEL_H

#include <vector>
#include "../imported/eigen/Eigen/Dense"
using namespace Eigen;
class Model {
 private:
  std::vector<Vector3f> verts_;
  std::vector<std::vector<int> > faces_;
 public:
  Model(const char *filename);
  ~Model();
  int nverts();
  int nfaces();
  Vector3f vert(int i);
  std::vector<int> face(int idx);};

#endif //SOFTRENDER_MODEL_H
