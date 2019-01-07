////
//// Created by Macx on 2018/12/19.
////
//
//#ifndef SOFTRENDER_SOFTGL_H
//#define SOFTRENDER_SOFTGL_H
//
//#include "tgaimage.h"
//#include "../imported/eigen/Eigen/Dense"
//#include "../imported/eigen/Eigen/Geometry"
//
//using Eigen::Vector2i;
//using Eigen::Vector2f;
//using Eigen::Vector3f;
//using Eigen::Vector4f;
//using Eigen::Matrix4f;
//
//void viewport(int x, int y, int w, int h);
//void projection(float coeff=0.f); // coeff = -1/c
//void lookat(Vector3f eye, Vector3f center, Vector3f up);
//
//struct IShader {
//  virtual ~IShader();
//  virtual Vector4f vertex(int iface, int nthvert) = 0;
//  virtual bool fragment(Vector3f bar, TGAColor &color) = 0;
//};
//
//void triangle(Vector4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer);
//
//#endif //SOFTRENDER_SOFTGL_H
