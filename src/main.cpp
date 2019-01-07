//
// Created by Macx on 2018/10/6.
//

#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>

#include "../imported/eigen/Eigen/Dense"
#include "../imported/eigen/Eigen/Geometry"
#include "Model.h"

#include <GL/glcorearb.h>

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793f
#define INFINITY 1e8
#endif

#include <SDL2/SDL.h>
#include <zconf.h>
#include <iostream>

const int width = 800;
const int height = 800;
const int depth = 255;

using Eigen::Vector3f;
using Eigen::Matrix4f;

SDL_PixelFormat *pixFormat = SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888);

Uint32 WHITE = SDL_MapRGBA(pixFormat, 255, 255, 255, 255);
Uint32 BLACK = SDL_MapRGBA(pixFormat, 0, 0, 0, 255);
Uint32 RED = SDL_MapRGBA(pixFormat, 255, 0, 0, 255);
Uint32 GREEN = SDL_MapRGBA(pixFormat, 0, 255, 0, 255);
Uint32 BLUE = SDL_MapRGBA(pixFormat, 0, 0, 255, 255);

Vector3f light_dir(1, -1, 1);
Vector3f eye(1, 1, 3);
Vector3f center(0, 0, 0);

Model *model = NULL;
SDL_Window *gWindow = NULL;
SDL_Renderer *gRender = NULL;
SDL_Texture *gTexture = NULL;

void init() {
  SDL_Init(SDL_INIT_VIDEO);
  gWindow =
      SDL_CreateWindow("SDL", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                       width, height, SDL_WINDOW_SHOWN);

  gRender = SDL_CreateRenderer(
      gWindow, -1, SDL_RENDERER_SOFTWARE | SDL_RENDERER_TARGETTEXTURE);

  gTexture = SDL_CreateTexture(gRender, SDL_PIXELFORMAT_RGBA8888,
                               SDL_TEXTUREACCESS_STREAMING, width, height);
}

void close() {
  SDL_DestroyRenderer(gRender);
  SDL_DestroyTexture(gTexture);
  SDL_DestroyWindow(gWindow);
  SDL_Quit();
}

void Clear(Uint32 *pix, Uint32 color) {
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      pix[i * width + j] = color;
    }
  }
}

void put_pixel(Uint32 *pix, Uint32 color, int x, int y) {
  pix[(height - y - 1) * width + x] = color;
}

void Line(int x0, int y0, int x1, int y1, Uint32 *pix, Uint32 color) {
  bool steep = false;
  if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
    std::swap(x0, y0);
    std::swap(x1, y1);
    steep = true;
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  for (int x = x0; x <= x1; x++) {
    float t = (x - x0) / (float) (x1 - x0);
    int y = y0 * (1. - t) + y1 * t;
    if (steep) {
      put_pixel(pix, color, y, x);
    } else {
      put_pixel(pix, color, x, y);
    }
  }
}

void Triangle(Vector2i t0, Vector2i t1, Vector2i t2, Uint32 *pix, Uint32 color) {
  if (t0.y() == t1.y() && t0.y() == t2.y()) return; // i dont care about degenerate triangles
  if (t0.y() > t1.y()) std::swap(t0, t1);
  if (t0.y() > t2.y()) std::swap(t0, t2);
  if (t1.y() > t2.y()) std::swap(t1, t2);
  int total_height = t2.y() - t0.y();
  for (int i = 0; i < total_height; i++) {
    bool second_half = i > t1.y() - t0.y() || t1.y() == t0.y();
    int segment_height = second_half ? t2.y() - t1.y() : t1.y() - t0.y();
    float alpha = (float) i / total_height;
    float beta = (float) (i - (second_half ? t1.y() - t0.y() : 0))
        / segment_height; // be careful: with above conditions no division by zero here
    Vector2i A = t0 + ((t2 - t0).cast<float>() * alpha).cast<int>();
    Vector2i B = second_half ? t1 + ((t2 - t1).cast<float>() * beta).cast<int>() : t0 + ((t1 - t0).cast<float>() *
        beta).cast<int>();
    if (A.x() > B.x()) std::swap(A, B);
    for (int j = A.x(); j <= B.x(); j++) {
      put_pixel(pix, color, j, t0.y() + i); // attention, due to int casts t0.y()+i != A.y
    }
  }
}

Vector3f barycentric(Vector3i A, Vector3i B, Vector3i C, Vector3i P) {
  Vector3i s[2];
  for (int i = 2; i--;) {
    s[i][0] = C[i] - A[i];
    s[i][1] = B[i] - A[i];
    s[i][2] = A[i] - P[i];
  }
  Vector3f u = (s[0].cross(s[1])).cast<float>();
  if (std::abs(u[2]) != 0) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
  {
    return Vector3f(1.f - (u.x() + u.y()) / u.z(), u.y() / u.z(), u.x() / u.z());
  }
  return Vector3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

int Det(Vector3i A, Vector3i B, Vector3i C)
{
  return (B.x()-A.x())*(C.y()-A.y())-(B.y()-A.y())*(C.x()-A.x());
}

void TriangleFast(Vector3i *pts, int *zbuffer, Uint32 *pix, Uint32 *colors) {
  Vector2i bboxmin(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
  Vector2i bboxmax(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
  Vector2i clamp(width - 1, height - 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bboxmin[j] = std::max(0, std::min(bboxmin[j], pts[i][j]));
      bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
    }
  }
  float Den = Det(pts[0],pts[1],pts[2]);

  Vector3i P;
  float z_inv_p;
  float z_p;
  for (P.x() = bboxmin.x(); P.x() <= bboxmax.x(); P.x()++) {
    for (P.y() = bboxmin.y(); P.y() <= bboxmax.y(); P.y()++) {
      float NonI = Det(pts[0],P,pts[2]);
      float NonJ = Det(pts[0],pts[1],P);
      float I = NonI / Den;
      float J = NonJ / Den;
      float IJ1 = 1-I-J;
      if(I < 0 ||J <0 ||IJ1<0) continue;
      P.z() = 0;
      float z_inv_0 =1.0/pts[0][2];
      float z_inv_1 =1.0/pts[1][2];
      float z_inv_2 =1.0/pts[2][2];

      z_inv_p = IJ1*z_inv_0 + I*z_inv_1 + J*z_inv_2;
      assert(z_inv_p > 1e-8);
      z_p = 1.0 / z_inv_p;
      Uint8 r_add = 0, g_add = 0, b_add = 0, a_add = 0;
      {
        Uint8 r[3], g[3], b[3], a[3];
        for (int i = 0; i < 3; i++) {
          SDL_GetRGBA(colors[i], pixFormat, &r[i], &g[i], &b[i], &a[i]);
        }
        r_add = z_p*(IJ1*z_inv_0*r[0]+I*z_inv_1*r[1]+J*z_inv_2*r[2]);
        g_add = z_p*(IJ1*z_inv_0*g[0]+I*z_inv_1*g[1]+J*z_inv_2*g[2]);
        b_add = z_p*(IJ1*z_inv_0*b[0]+I*z_inv_1*b[1]+J*z_inv_2*b[2]);
      }
      P.z() = z_p;
      Uint32 color = SDL_MapRGBA(pixFormat, r_add, g_add, b_add, a_add);
      int idx = P.x() + P.y() * width;
      if (zbuffer[idx] < P.z()) {
        zbuffer[idx] = P.z();
        put_pixel(pix, color, P.x(), P.y());
      }
    }
  }
}

void TriangleFast2(Vector3i *pts, int *zbuffer, Uint32 *pix, Uint32 *colors) {
  Vector2i bboxmin(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
  Vector2i bboxmax(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
  Vector2i clamp(width - 1, height - 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bboxmin[j] = std::max(0, std::min(bboxmin[j], pts[i][j]));
      bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
    }
  }

  Uint8 r[3], g[3], b[3], a[3];
  for (int i = 0; i < 3; i++) {
    SDL_GetRGBA(colors[i], pixFormat, &r[i], &g[i], &b[i], &a[i]);
  }

  float z_inv_0 =1.0/pts[0][2];
  float z_inv_1 =1.0/pts[1][2];
  float z_inv_2 =1.0/pts[2][2];

  float y_0 = pts[0][1];
  float y_1 = pts[1][1];
  float y_2 = pts[2][1];

  float x_0 = pts[0][0];
  float x_1 = pts[1][0];
  float x_2 = pts[2][0];

  float Den = Det(pts[0],pts[1],pts[2]);

  float D_Zpinv_x = ((y_2 -y_0)*(z_inv_1-z_inv_0)+(y_0-y_1)*(z_inv_2-z_inv_0))/Den;
  float D_Zpinv_y = ((x_0 -x_2)*(z_inv_1-z_inv_0)+(x_1-x_0)*(z_inv_2-z_inv_0))/Den;

  float D_Zpinv_x_r = ((y_2 -y_0)*(z_inv_1*r[1]-z_inv_0*r[0])+(y_0-y_1)*(z_inv_2*r[2]-z_inv_0*r[0]))/Den;
  float D_Zpinv_y_r = ((x_0 -x_2)*(z_inv_1*r[1]-z_inv_0*r[0])+(x_1-x_0)*(z_inv_2*r[2]-z_inv_0*r[0]))/Den;

  float D_Zpinv_x_g = ((y_2 -y_0)*(z_inv_1*g[1]-z_inv_0*g[0])+(y_0-y_1)*(z_inv_2*g[2]-z_inv_0*g[0]))/Den;
  float D_Zpinv_y_g = ((x_0 -x_2)*(z_inv_1*g[1]-z_inv_0*g[0])+(x_1-x_0)*(z_inv_2*g[2]-z_inv_0*g[0]))/Den;

  float D_Zpinv_x_b = ((y_2 -y_0)*(z_inv_1*b[1]-z_inv_0*b[0])+(y_0-y_1)*(z_inv_2*b[2]-z_inv_0*b[0]))/Den;
  float D_Zpinv_y_b = ((x_0 -x_2)*(z_inv_1*b[1]-z_inv_0*b[0])+(x_1-x_0)*(z_inv_2*b[2]-z_inv_0*b[0]))/Den;


  float z_p;
  float z_inv_p;

  z_inv_p = z_inv_0 +(bboxmin.x() - x_0)*D_Zpinv_x +(bboxmin.y() - y_0)*D_Zpinv_y;
  z_p = 1.0/z_inv_p;
  float r_i_p =  (r[0]*z_inv_0 +(bboxmin.x()-x_0)*D_Zpinv_x_r +(bboxmin.y()-y_0)*D_Zpinv_y_r);
  float g_i_p =  (g[0]*z_inv_0 +(bboxmin.x()-x_0)*D_Zpinv_x_g +(bboxmin.y()-y_0)*D_Zpinv_y_g);
  float b_i_p =  (b[0]*z_inv_0 +(bboxmin.x()-x_0)*D_Zpinv_x_b +(bboxmin.y()-y_0)*D_Zpinv_y_b);

  Vector3i P;
  for (P.x() = bboxmin.x(); P.x() <= bboxmax.x(); P.x()++) {
    z_inv_p += D_Zpinv_x;
    r_i_p += D_Zpinv_x_r;
    g_i_p += D_Zpinv_x_g;
    b_i_p += D_Zpinv_x_b;
    float z_inv_p_base = z_inv_p;
    float r_i_p_base = r_i_p;
    float g_i_p_base = g_i_p;
    float b_i_p_base = b_i_p;

    for (P.y() = bboxmin.y(); P.y() <= bboxmax.y(); P.y()++) {
//      float NonI = Det(pts[0],P,pts[2]);
//      float NonJ = Det(pts[0],pts[1],P);
//      float I = NonI / Den;
//      float J = NonJ / Den;
//      float IJ1 = 1-I-J;
//      if(I < 0 ||J <0 ||IJ1<0) continue;
      P.z() = 0;

      z_inv_p_base +=  D_Zpinv_y;
      r_i_p_base +=  D_Zpinv_y_r;
      g_i_p_base +=  D_Zpinv_y_g;
      b_i_p_base +=  D_Zpinv_y_b;
      z_p = 1.0/z_inv_p_base;
      P.z() = z_p;
      Uint32 color = SDL_MapRGBA(pixFormat, r_i_p_base*z_p, g_i_p_base*z_p, b_i_p_base*z_p, 255);
      int idx = P.x() + P.y() * width;
      if (zbuffer[idx] < P.z()) {
        zbuffer[idx] = P.z();
        put_pixel(pix, color, P.x(), P.y());
      }
    }
  }
}


void Triangle(Vector3i *pts, int *zbuffer, Uint32 *pix, Uint32 *colors) {
  Vector2i bboxmin(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
  Vector2i bboxmax(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
  Vector2i clamp(width - 1, height - 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bboxmin[j] = std::max(0, std::min(bboxmin[j], pts[i][j]));
      bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
    }
  }
  Vector3i P;
  float z_inv;
  for (P.x() = bboxmin.x(); P.x() <= bboxmax.x(); P.x()++) {
    for (P.y() = bboxmin.y(); P.y() <= bboxmax.y(); P.y()++) {
      Vector3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
      if (bc_screen.x() < 0 || bc_screen.y() < 0 || bc_screen.z() < 0) continue;
      P.z() = 0;
      z_inv = 0;
      for (int i = 0; i < 3; i++) z_inv += 1.0 / pts[i][2] * bc_screen[i];
      Uint8 r_add = 0, g_add = 0, b_add = 0, a_add = 0;
      {
        Uint8 r, g, b, a;
        for (int i = 0; i < 3; i++) {
          SDL_GetRGBA(colors[i], pixFormat, &r, &g, &b, &a);
          float pfx = bc_screen[i] / pts[i][2] / z_inv;
          r_add += r * pfx;
          g_add += g * pfx;
          b_add += b * pfx;
          a_add += a * pfx;
        }
      }
      P.z() = 1.0 / z_inv;
      Uint32 color = SDL_MapRGBA(pixFormat, r_add, g_add, b_add, a_add);
      int idx = P.x() + P.y() * width;
      if (zbuffer[idx] < P.z()) {
        zbuffer[idx] = P.z();
        put_pixel(pix, color, P.x(), P.y());
      }
    }
  }
}

template<typename T>
class Device{
 public:
  void sr_glViewport(GLint x,
                  GLint y,
                  GLsizei width,
                  GLsizei height) {
    viewPort.m_x = x;
    viewPort.m_y = y;
    viewPort.m_width = width;
    viewPort.m_height = height;
  }

  void sr_glDepthRange(T nearVal,
                    T farVal) {
    viewPort.m_nearVal = nearVal;
    viewPort.m_farVal = farVal;
  }
 private:
  struct ViewPort{
    GLint m_x = 0;
    GLint m_y = 0;
    GLsizei m_width = 1;
    GLsizei m_height = 1;
    T m_nearVal = 0.0;
    T m_farVal = 1.0;
  };
 protected:
  ViewPort viewPort;
};


Matrix4f viewport(int x, int y, int w, int h) {
  Matrix4f m;
  m.setIdentity();
  m(0, 3) = x + w / 2.f;
  m(1, 3) = y + h / 2.f;
  m(2, 3) = depth / 2.f;

  m(0, 0) = w / 2.f;
  m(1, 1) = h / 2.f;
  m(2, 2) = depth / 2.f;
  return m;
}

Vector3f world2screen(Vector3f v) {
  return Vector3f(int((v.x() + 1.) * width / 2. + .5), int((v.y() + 1.) * height / 2. + .5), v.z());
}

Matrix4f lookat(Vector3f eye, Vector3f center, Vector3f up) {
  Vector3f z = (eye - center);
  z.normalize();
  Vector3f x = up.cross(z);
  x.normalize();
  Vector3f y = z.cross(x);
  y.normalize();
  Matrix4f res;
  res.setIdentity();
  for (int i = 0; i < 3; i++) {
    res(0, i) = x[i];
    res(1, i) = y[i];
    res(2, i) = z[i];
    res(i, 3) = -center[i];
  }
  return res;
}

int main(int argc, char **argv) {
  if (2 == argc) {
    model = new Model(argv[1]);
  } else {
    model = new Model("../../obj/african_head.obj");
  }

//add device
  Device<GLfloat> m_device;
  m_device.sr_glViewport(0,0,800,800);
  m_device.sr_glDepthRange(0,1);

  int *zbuffer = new int[width * height];
  light_dir.normalize();
  void *pix;
  int pitch;

  init();

  int x_current(0);
  int y_current(0);

  // Main loop flag
  bool quit = false;

  // Event handler
  SDL_Event e;

  float deltaX(1), deltaY(1), deltaZ(3);

  eye  << deltaX, deltaY, deltaZ;

  // While application is running
  while (!quit) {
    // Handle events on queue
    while (SDL_PollEvent(&e) != 0) {
      // User requests quit
      if (e.type == SDL_QUIT || e.key.keysym.sym == SDLK_ESCAPE) {
        quit = true;
      }
      if (e.key.keysym.sym == SDLK_UP) {
        deltaY += 0.2f;
        break;
      }
      if (e.key.keysym.sym == SDLK_DOWN) {
        deltaY -= 0.2f;
        break;
      }
      if (e.key.keysym.sym == SDLK_RIGHT) {
        deltaX += 0.2f;
        break;
      }
      if (e.key.keysym.sym == SDLK_LEFT) {
        deltaX -= 0.2f;
        break;
      }
      if (e.key.keysym.sym == SDLK_w) {
        deltaZ -= 0.2f;
        break;
      }
      if (e.key.keysym.sym == SDLK_s) {
        deltaZ += 0.2f;
        break;
      }
      if (e.type == SDL_MOUSEMOTION || e.button.button == SDL_BUTTON_LEFT) {
        x_current = e.motion.x;
        y_current = e.motion.y;
      }
    }

    SDL_LockTexture(gTexture, NULL, &pix, &pitch);
    // Render begin

    // Clear color
    Clear((Uint32 *) pix, BLACK);

    // Eye and center position
    eye  << deltaX, deltaY, deltaZ;

    Matrix4f ModelView = lookat(eye, center, Vector3f(0, 1, 0));
    Matrix4f Projection;
    Projection.setIdentity();
    Matrix4f ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    Projection(3, 2) = -1.f / (eye - center).norm();
    std::cerr << "ModelView" << std::endl;
    std::cerr << ModelView << std::endl;
    std::cerr << "Projection" << std::endl;
    std::cerr << Projection << std::endl;
    std::cerr << "ViewPort" << std::endl;
    std::cerr << ViewPort << std::endl;
    Matrix4f z = ViewPort * Projection * ModelView;
    std::cerr << z << std::endl;

    for (int i = width * height; i--; zbuffer[i] = std::numeric_limits<int>::min());
    // Render face
//    for (int i = 0; i < model->nfaces(); i++) {
//      std::vector<int> face = model->face(i);
//      Vector3i screen_coords[3];
//      Vector3f world_coords[3];
//      Uint32 colors[3];
//      Vector3f n_v[3];
//      float intensity[3];
//
//      // load texture data
//      for (int j = 0; j < 3; ++j) {
//        Vector3f v = model->vert(face[j]);
//        n_v[j] = model->normal(i, j);
//
//        Vector4f v_;
//        v_ << v, 1;
//        Vector4f v_temp;
//        v_temp = ViewPort * Projection * ModelView * v_;
//        v_temp = v_temp / v_temp.w();
//        screen_coords[j] = (v_temp).cast<int>().block(0, 0, 3, 1);
//
//        world_coords[j] = v;
//        intensity[j] = -1.0f * (n_v[j].dot(light_dir));
//        if (intensity[j] > 0) {
//          colors[j] = SDL_MapRGBA(pixFormat,
//                                  static_cast<Uint8>(model->diffuse(model->uv(i, j))[2] * intensity[j]),
//                                  static_cast<Uint8>(model->diffuse(model->uv(i, j))[1] * intensity[j]),
//                                  static_cast<Uint8>(model->diffuse(model->uv(i, j))[0] * intensity[j]),
//                                  model->diffuse(model->uv(i, j))[3]);
//        } else {
//          colors[j] = SDL_MapRGBA(pixFormat,
//                                  model->diffuse(model->uv(i, j))[2],
//                                  model->diffuse(model->uv(i, j))[1],
//                                  model->diffuse(model->uv(i, j))[0],
//                                  model->diffuse(model->uv(i, j))[3]);
//        }
//
//      }
//
//      // back face culling
//      Vector3i AB = screen_coords[0]-screen_coords[1];
//      Vector3i AC = screen_coords[0]-screen_coords[2];
//      Vector3i N = AC.cross(AB);
//      if(N.z()>0){
//        continue;
//      }
//
//      // tessellation
//      Triangle(screen_coords, zbuffer, (Uint32 *) pix, colors);
//    }

    Vector3f world_coords[3];
    world_coords[0] << 0, 0, 0.99;
    world_coords[1] << 1, 0.5, 0.99;
    world_coords[2] << 0.5, 1, -0.99;

    Vector3i screen_coords[3];

    Uint32 colors[3];
    colors[0] = RED;
    colors[1] = GREEN;
    colors[2] = BLUE;
    for (int j = 0; j < 3; ++j) {
      Vector4f v_;
      v_ << world_coords[j].cast<float>(), 1;
      Vector4f v_temp;
      v_temp = ViewPort * Projection * ModelView * v_;
      v_temp = v_temp / v_temp.w();
      screen_coords[j] = (v_temp).cast<int>().block(0, 0, 3, 1);
    }

    // tessellation
    TriangleFast2(screen_coords, zbuffer, (Uint32 *) pix, colors);

    // Render end
    SDL_UnlockTexture(gTexture);
    SDL_RenderCopy(gRender, gTexture, NULL, NULL);
    SDL_RenderPresent(gRender);
    usleep(1000);
  }
  close();
}
