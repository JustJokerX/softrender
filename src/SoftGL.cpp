////
//// Created by Macx on 2018/12/19.
////
//
//#include "SoftGL.h"
//
//#include <cmath>
//#include <limits>
//#include <cstdlib>
//
//Matrix4f ModelView;
//Matrix4f Viewport;
//Matrix4f Projection;
//
//IShader::~IShader() {}
//
//void viewport(int x, int y, int w, int h) {
//  Viewport.setIdentity();
//  Viewport[0,3] = x+w/2.f;
//  Viewport[1,3] = y+h/2.f;
//  Viewport[2,3] = 255.f/2.f;
//  Viewport[0,0] = w/2.f;
//  Viewport[1,1] = h/2.f;
//  Viewport[2,2] = 255.f/2.f;
//}
//
//void projection(float coeff) {
//  Projection.setIdentity();
//  Projection[3,2] = coeff;
//}
//
//void lookat(Vector3f eye, Vector3f center, Vector3f up) {
//  Vector3f z = (eye - center);
//  z.normalize();
//  Vector3f x = up.cross(z);
//  x.normalize();
//  Vector3f y = z.cross(x);
//  y.normalize();
//  Matrix4f res;
//  ModelView.setIdentity();
//  for (int i = 0; i < 3; i++) {
//    ModelView(0, i) = x[i];
//    ModelView(1, i) = y[i];
//    ModelView(2, i) = z[i];
//    ModelView(i, 3) = -center[i];
//  }
//}
//
//Vector3f barycentric(Vector2f A, Vector2f B, Vector2f C, Vector2f P) {
//  Vector3f s[2];
//  for (int i=2; i--; ) {
//    s[i][0] = C[i]-A[i];
//    s[i][1] = B[i]-A[i];
//    s[i][2] = A[i]-P[i];
//  }
//  Vector3f u = s[0].cross(s[1]);
//  if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
//    return Vector3f(1.f-(u.x()+u.y())/u.z(), u.y()/u.z(), u.x()/u.z());
//  return Vector3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
//}
//
//void triangle(Vector4f *pts, IShader &shader, TGAImage &image, TGAImage &zbuffer) {
//  Vector2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
//  Vector2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
//  for (int i=0; i<3; i++) {
//    for (int j=0; j<2; j++) {
//      bboxmin[j] = std::min(bboxmin[j], pts[i][j]/pts[i][3]);
//      bboxmax[j] = std::max(bboxmax[j], pts[i][j]/pts[i][3]);
//    }
//  }
//  Vector2i P;
//  TGAColor color;
//  for (P.x()=bboxmin.x(); P.x()<=bboxmax.x(); P.x()++) {
//    for (P.y()=bboxmin.y(); P.y()<=bboxmax.y(); P.y()++) {
//      Vector3f c = barycentric(proj<2>(pts[0]/pts[0][3]), proj<2>(pts[1]/pts[1][3]), proj<2>(pts[2]/pts[2][3]), proj<2>(P));
//      float z = pts[0][2]*c.x() + pts[1][2]*c.y() + pts[2][2]*c.z();
//      float w = pts[0][3]*c.x() + pts[1][3]*c.y() + pts[2][3]*c.z();
//      int frag_depth = std::max(0, std::min(255, int(z/w+.5)));
//      if (c.x()<0 || c.y()<0 || c.z()<0 || zbuffer.get(P.x(), P.y())[0]>frag_depth) continue;
//      bool discard = shader.fragment(c, color);
//      if (!discard) {
//        zbuffer.set(P.x(), P.y, TGAColor(frag_depth));
//        image.set(P.x(), P.y, color);
//      }
//    }
//  }
//}