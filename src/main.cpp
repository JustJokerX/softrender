//
// Created by Macx on 2018/10/6.
//

#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstddef>
#include <limits>
#include <algorithm>
#include <time.h>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
#include <iostream>
#include "Model.h"
#include <ofxProfiler.h>
#include <GL/glcorearb.h>
#include "Math.hpp"
#include "Constants.hpp"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793f
#define INFINITY 1e8
#endif

#if defined __linux__ || defined __APPLE__
#include <SDL.h>
#include <zconf.h>
#else
#include "SDL.h"
#undef min
#undef max
#endif

const S32 width = 800;
const S32 height = 800;
const S32 depth = 255;

//using Eigen::Vec3f;
//using Eigen::Mat4f;
using namespace FW;

SDL_PixelFormat *pixFormat = SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888);

U32 WHITE = SDL_MapRGBA(pixFormat, 255, 255, 255, 255);
U32 BLACK = SDL_MapRGBA(pixFormat, 0, 0, 0, 255);
U32 RED = SDL_MapRGBA(pixFormat, 255, 0, 0, 255);
U32 GREEN = SDL_MapRGBA(pixFormat, 0, 255, 0, 255);
U32 BLUE = SDL_MapRGBA(pixFormat, 0, 0, 255, 255);

Vec3f light_dir(1, -1, 1);
Vec3f eye(1, 1, 3);
Vec3f center(0, 0, 0);

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

void Clear(U32 *pix, U32 color) {
  for (S32 i = 0; i < height; ++i) {
    for (S32 j = 0; j < width; ++j) {
      pix[i * width + j] = color;
    }
  }
}

void put_pixel(U32 *pix, U32 color, S32 x, S32 y) {
  pix[(height - y - 1) * width + x] = color;
}

void Line(S32 x0, S32 y0, S32 x1, S32 y1, U32 *pix, U32 color) {
  bool steep = false;
  if (FW::abs(x0 - x1) < FW::abs(y0 - y1)) {
    FW::swap(x0, y0);
    FW::swap(x1, y1);
    steep = true;
  }
  if (x0 > x1) {
    FW::swap(x0, x1);
    FW::swap(y0, y1);
  }

  for (S32 x = x0; x <= x1; x++) {
    F32 t = (x - x0) / (F32) (x1 - x0);
    S32 y = y0 * (1.f - t) + y1 * t;
    if (steep) {
      put_pixel(pix, color, y, x);
    } else {
      put_pixel(pix, color, x, y);
    }
  }
}

void Triangle(Vec2i t0, Vec2i t1, Vec2i t2, U32 *pix, U32 color) {
  if (t0.y == t1.y && t0.y == t2.y) return; // i dont care about degenerate triangles
  if (t0.y > t1.y) FW::swap(t0, t1);
  if (t0.y > t2.y) FW::swap(t0, t2);
  if (t1.y > t2.y) FW::swap(t1, t2);
  S32 total_height = t2.y - t0.y;
  for (S32 i = 0; i < total_height; i++) {
    bool second_half = i > t1.y - t0.y || t1.y == t0.y;
    S32 segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
    F32 alpha = (F32) i / total_height;
    F32 beta = (F32) (i - (second_half ? t1.y - t0.y : 0))
        / segment_height; // be careful: with above conditions no division by zero here
    Vec2i A = t0 + ((t2 - t0) * alpha);
    Vec2i B = second_half ? t1 + ((t2 - t1) * beta) : t0 + ((t1 - t0) *
        beta);
    if (A.x > B.x) FW::swap(A, B);
    for (S32 j = A.x; j <= B.x; j++) {
      put_pixel(pix, color, j, t0.y + i); // attention, due to S32 casts t0.y+i != A.y
    }
  }
}

Vec3f barycentric(Vec3i A, Vec3i B, Vec3i C, Vec3i P) {
  Vec3i s[2];
  for (S32 i = 2; i--;) {
    s[i][0] = C[i] - A[i];
    s[i][1] = B[i] - A[i];
    s[i][2] = A[i] - P[i];
  }
  Vec3f u = FW::cross((Vec3f)s[0], (Vec3f)s[1]);
  if (FW::abs(u[2]) != 0) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
  {
    return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
  }
  return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

F32 Det(Vec3f A, Vec3f B, Vec3f C) {
  return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

void TriangleFast2(Vec3f *pts, S32 *zbuffer, U32 *pix, U32 *colors) {
  Vec2i bboxmin(FW_S32_MAX, FW_S32_MAX);
  Vec2i bboxmax(FW_S32_MIN, FW_S32_MIN);
  Vec2i clamp(width - 1, height - 1);

  for (S32 i = 0; i < 3; i++) {
    for (S32 j = 0; j < 2; j++) {
      bboxmin[j] = FW::max(0, FW::min(bboxmin[j], (S32) (pts[i][j] + 0.5)));
      bboxmax[j] = FW::min(clamp[j], FW::max(bboxmax[j], (S32) (pts[i][j] + 0.5)));
    }
  }

  U8 r[3], g[3], b[3], a[3];
  for (S32 i = 0; i < 3; i++) {
    SDL_GetRGBA(colors[i], pixFormat, &r[i], &g[i], &b[i], &a[i]);
  }

  F32 z_inv_0 = 1.0f / pts[0][2];
  F32 z_inv_1 = 1.0f / pts[1][2];
  F32 z_inv_2 = 1.0f / pts[2][2];

  F32 y_0 = pts[0][1];
  F32 y_1 = pts[1][1];
  F32 y_2 = pts[2][1];

  F32 x_0 = pts[0][0];
  F32 x_1 = pts[1][0];
  F32 x_2 = pts[2][0];

  Vec2f n_pq(pts[0].y - pts[1].y, pts[1].x - pts[0].x);
  Vec2f n_qr(pts[1].y - pts[2].y, pts[2].x - pts[1].x);
  Vec2f n_rp(pts[2].y - pts[0].y, pts[0].x - pts[2].x);
  Vec2f v_p = pts[0].getXY();
  Vec2f v_q = pts[1].getXY();
  Vec2f v_r = pts[2].getXY();

  F32 c_pq = -1.f * (n_pq.dot(v_p));
  F32 c_qr = -1.f * (n_qr.dot(v_q));
  F32 c_rp = -1.f * (n_rp.dot(v_r));

  F32 Den = Det(pts[0], pts[1], pts[2]);

  F32 D_Zpinv_x = ((y_2 - y_0) * (z_inv_1 - z_inv_0) + (y_0 - y_1) * (z_inv_2 - z_inv_0)) / Den;
  F32 D_Zpinv_y = ((x_0 - x_2) * (z_inv_1 - z_inv_0) + (x_1 - x_0) * (z_inv_2 - z_inv_0)) / Den;

  F32 D_Zpinv_x_r =
      ((y_2 - y_0) * (z_inv_1 * r[1] - z_inv_0 * r[0]) + (y_0 - y_1) * (z_inv_2 * r[2] - z_inv_0 * r[0])) / Den;
  F32 D_Zpinv_y_r =
      ((x_0 - x_2) * (z_inv_1 * r[1] - z_inv_0 * r[0]) + (x_1 - x_0) * (z_inv_2 * r[2] - z_inv_0 * r[0])) / Den;

  F32 D_Zpinv_x_g =
      ((y_2 - y_0) * (z_inv_1 * g[1] - z_inv_0 * g[0]) + (y_0 - y_1) * (z_inv_2 * g[2] - z_inv_0 * g[0])) / Den;
  F32 D_Zpinv_y_g =
      ((x_0 - x_2) * (z_inv_1 * g[1] - z_inv_0 * g[0]) + (x_1 - x_0) * (z_inv_2 * g[2] - z_inv_0 * g[0])) / Den;

  F32 D_Zpinv_x_b =
      ((y_2 - y_0) * (z_inv_1 * b[1] - z_inv_0 * b[0]) + (y_0 - y_1) * (z_inv_2 * b[2] - z_inv_0 * b[0])) / Den;
  F32 D_Zpinv_y_b =
      ((x_0 - x_2) * (z_inv_1 * b[1] - z_inv_0 * b[0]) + (x_1 - x_0) * (z_inv_2 * b[2] - z_inv_0 * b[0])) / Den;

  F32 z_p;
  F32 z_inv_p;

  z_inv_p = z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x + (bboxmin.y - y_0) * D_Zpinv_y;
  z_p = 1.0f / z_inv_p;
  F32 r_i_p = (r[0] * z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x_r + (bboxmin.y - y_0) * D_Zpinv_y_r);
  F32 g_i_p = (g[0] * z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x_g + (bboxmin.y - y_0) * D_Zpinv_y_g);
  F32 b_i_p = (b[0] * z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x_b + (bboxmin.y - y_0) * D_Zpinv_y_b);

  Vec3i P;
  for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
    F32 z_inv_p_base = z_inv_p;
    F32 r_i_p_base = r_i_p;
    F32 g_i_p_base = g_i_p;
    F32 b_i_p_base = b_i_p;
    z_inv_p += D_Zpinv_x;
    r_i_p += D_Zpinv_x_r;
    g_i_p += D_Zpinv_x_g;
    b_i_p += D_Zpinv_x_b;
    for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
      P.z = 0;

      z_p = 1.0f / z_inv_p_base;
      P.z = z_p;
      U32 color = SDL_MapRGBA(pixFormat, r_i_p_base * z_p, g_i_p_base * z_p, b_i_p_base * z_p, 255);

      z_inv_p_base += D_Zpinv_y;
      r_i_p_base += D_Zpinv_y_r;
      g_i_p_base += D_Zpinv_y_g;
      b_i_p_base += D_Zpinv_y_b;

      Vec2f s = P.getXY();
      F32 e_pq_s = n_pq.dot(s) + c_pq;
      if (e_pq_s >= 0) {
        F32 e_qr_s = n_qr.dot(s) + c_qr;
        if (e_qr_s >= 0) {
          F32 e_rp_s = n_rp.dot(s) + c_rp;
          if (e_rp_s < 0)
            continue;
        } else continue;
      } else continue;

      S32 idx = P.x + P.y * width;
      if (zbuffer[idx] < P.z) {
        zbuffer[idx] = P.z;
        put_pixel(pix, color, P.x, P.y);
      }
    }
  }
}
#if 0
void Triangle(Vec3i *pts, S32 *zbuffer, U32 *pix, U32 *colors) {
  Vec2i bboxmin(FW::numeric_limits<S32>::max, FW::numeric_limits<S32>::max);
  Vec2i bboxmax(-FW::numeric_limits<S32>::max, -FW::numeric_limits<S32>::max);
  Vec2i clamp(width - 1, height - 1);
  for (S32 i = 0; i < 3; i++) {
    for (S32 j = 0; j < 2; j++) {
      bboxmin[j] = FW::max(0, FW::min(bboxmin[j], pts[i][j]));
      bboxmax[j] = FW::min(clamp[j], FW::max(bboxmax[j], pts[i][j]));
    }
  }
  Vec3i P;
  F32 z_inv;
  for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
    for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
      Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
      if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
      P.z = 0;
      z_inv = 0;
      for (S32 i = 0; i < 3; i++) z_inv += 1.0 / pts[i][2] * bc_screen[i];
      U8 r_add = 0, g_add = 0, b_add = 0, a_add = 0;
      {
        U8 r, g, b, a;
        for (S32 i = 0; i < 3; i++) {
          SDL_GetRGBA(colors[i], pixFormat, &r, &g, &b, &a);
          F32 pfx = bc_screen[i] / pts[i][2] / z_inv;
          r_add += r * pfx;
          g_add += g * pfx;
          b_add += b * pfx;
          a_add += a * pfx;
        }
      }
      P.z = 1.0 / z_inv;
      U32 color = SDL_MapRGBA(pixFormat, r_add, g_add, b_add, a_add);
      S32 idx = P.x + P.y * width;
      if (zbuffer[idx] < P.z) {
        zbuffer[idx] = P.z;
        put_pixel(pix, color, P.x, P.y);
      }
    }
  }
}
#endif // 0

//template<class A, class B>
//inline A lerp(const A &a, const A &b, const B &t) { return (A) (a * ((B) 1 - t) + b * t); }

Vec3i setupPleq(const Vec3f& values, const Vec2i& v0, const Vec2i& d1, const Vec2i& d2, S32 area, int samplesLog2)
{
  F64 t0 = (F64)values.x;
  F64 t1 = (F64)values.y - t0;
  F64 t2 = (F64)values.z - t0;
  F64 xc = (t1 * (F64)d2.y - t2 * (F64)d1.y) / (F64)area;
  F64 yc = (t2 * (F64)d1.x - t1 * (F64)d2.x) / (F64)area;

  Vec2i center = (v0 * 2 + min(d1.x, d2.x, 0) + max(d1.x, d2.x, 0)) >> (CR_SUBPIXEL_LOG2 - samplesLog2 + 1);
  Vec2i vc = v0 - (center << (CR_SUBPIXEL_LOG2 - samplesLog2));

  Vec3i pleq;
  pleq.x = (U32)(S64)FW::floor(xc * exp2(CR_SUBPIXEL_LOG2 - samplesLog2) + 0.5);
  pleq.y = (U32)(S64)FW::floor(yc * exp2(CR_SUBPIXEL_LOG2 - samplesLog2) + 0.5);
  pleq.z = (U32)(S64)FW::floor(t0 - xc * (F64)vc.x - yc * (F64)vc.y + 0.5);
  pleq.z -= pleq.x * center.x + pleq.y * center.y;
  return pleq;
}

bool setupTriangle(Vec3f *pts) {

  Vec3f v0 = pts[0];
  Vec3f v1 = pts[1];
  Vec3f v2 = pts[2];

  Vec2i p0 = Vec2i(v0.x, v0.y);
  Vec2i p1 = Vec2i(v1.x, v1.y);
  Vec2i p2 = Vec2i(v2.x, v2.y);
  Vec2i d1 = p1 - p0;
  Vec2i d2 = p2 - p0;

  S32 area = d1.x * d2.y - d1.y * d2.x;

  if (area <= 0)
    return false;

  Vec3f zvert = FW::lerp(Vec3f(CR_DEPTH_MIN), Vec3f(CR_DEPTH_MIN), Vec3f(v0.z, v1.z, v2.z)* 0.5f + 0.5f);
//  zvert.print();
  return true;
}

void TriangleFast3(Vec3f *pts, S32 *zbuffer, U32 *pix, U32 *colors) {
  Vec2i bboxmin(FW_S32_MAX, FW_S32_MAX);
  Vec2i bboxmax(FW_S32_MIN, FW_S32_MIN);
  Vec2i clamp(width - 1, height - 1);

  for (S32 i = 0; i < 3; i++) {
    for (S32 j = 0; j < 2; j++) {
      bboxmin[j] = FW::max(0, FW::min(bboxmin[j], (S32) (pts[i][j] + 0.5)));
      bboxmax[j] = FW::min(clamp[j], FW::max(bboxmax[j], (S32) (pts[i][j] + 0.5)));
    }
  }

  U8 r[3], g[3], b[3], a[3];
  for (S32 i = 0; i < 3; i++) {
    SDL_GetRGBA(colors[i], pixFormat, &r[i], &g[i], &b[i], &a[i]);
  }

  F32 z_inv_0 = 1.0f / pts[0][2];
  F32 z_inv_1 = 1.0f / pts[1][2];
  F32 z_inv_2 = 1.0f / pts[2][2];

  F32 y_0 = pts[0][1];
  F32 y_1 = pts[1][1];
  F32 y_2 = pts[2][1];

  F32 x_0 = pts[0][0];
  F32 x_1 = pts[1][0];
  F32 x_2 = pts[2][0];

  Vec2f n_pq(pts[0].y - pts[1].y, pts[1].x - pts[0].x);
  Vec2f n_qr(pts[1].y - pts[2].y, pts[2].x - pts[1].x);
  Vec2f n_rp(pts[2].y - pts[0].y, pts[0].x - pts[2].x);
  Vec2f v_p = pts[0].getXY();
  Vec2f v_q = pts[1].getXY();
  Vec2f v_r = pts[2].getXY();

  F32 c_pq = -1.f * (n_pq.dot(v_p));
  F32 c_qr = -1.f * (n_qr.dot(v_q));
  F32 c_rp = -1.f * (n_rp.dot(v_r));
  setupTriangle(pts);
  F32 Den = Det(pts[0], pts[1], pts[2]);
  F32 Den_1 = 1.0f / Den;

  F32 D_Zpinv_x = ((y_2 - y_0) * (z_inv_1 - z_inv_0) + (y_0 - y_1) * (z_inv_2 - z_inv_0)) * Den_1;
  F32 D_Zpinv_y = ((x_0 - x_2) * (z_inv_1 - z_inv_0) + (x_1 - x_0) * (z_inv_2 - z_inv_0)) * Den_1;

  F32 D_Zpinv_x_r =
      ((y_2 - y_0) * (z_inv_1 * r[1] - z_inv_0 * r[0]) + (y_0 - y_1) * (z_inv_2 * r[2] - z_inv_0 * r[0])) * Den_1;
  F32 D_Zpinv_y_r =
      ((x_0 - x_2) * (z_inv_1 * r[1] - z_inv_0 * r[0]) + (x_1 - x_0) * (z_inv_2 * r[2] - z_inv_0 * r[0])) * Den_1;

  F32 D_Zpinv_x_g =
      ((y_2 - y_0) * (z_inv_1 * g[1] - z_inv_0 * g[0]) + (y_0 - y_1) * (z_inv_2 * g[2] - z_inv_0 * g[0])) * Den_1;
  F32 D_Zpinv_y_g =
      ((x_0 - x_2) * (z_inv_1 * g[1] - z_inv_0 * g[0]) + (x_1 - x_0) * (z_inv_2 * g[2] - z_inv_0 * g[0])) * Den_1;

  F32 D_Zpinv_x_b =
      ((y_2 - y_0) * (z_inv_1 * b[1] - z_inv_0 * b[0]) + (y_0 - y_1) * (z_inv_2 * b[2] - z_inv_0 * b[0])) * Den_1;
  F32 D_Zpinv_y_b =
      ((x_0 - x_2) * (z_inv_1 * b[1] - z_inv_0 * b[0]) + (x_1 - x_0) * (z_inv_2 * b[2] - z_inv_0 * b[0])) * Den_1;

  F32 z_p;
  F32 z_inv_p;

  z_inv_p = z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x + (bboxmin.y - y_0) * D_Zpinv_y;
  z_p = 1.0 / z_inv_p;
  F32 r_i_p = (r[0] * z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x_r + (bboxmin.y - y_0) * D_Zpinv_y_r);
  F32 g_i_p = (g[0] * z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x_g + (bboxmin.y - y_0) * D_Zpinv_y_g);
  F32 b_i_p = (b[0] * z_inv_0 + (bboxmin.x - x_0) * D_Zpinv_x_b + (bboxmin.y - y_0) * D_Zpinv_y_b);

  Vec3i P;
  for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
    F32 z_inv_p_base = z_inv_p;
    F32 r_i_p_base = r_i_p;
    F32 g_i_p_base = g_i_p;
    F32 b_i_p_base = b_i_p;

    z_inv_p += D_Zpinv_x;
    r_i_p += D_Zpinv_x_r;
    g_i_p += D_Zpinv_x_g;
    b_i_p += D_Zpinv_x_b;

    for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
      P.z = 0;

      z_p = 1.0 / z_inv_p_base;
      P.z = z_p;
      U32 color = SDL_MapRGBA(pixFormat, r_i_p_base * z_p, g_i_p_base * z_p, b_i_p_base * z_p, 255);

      z_inv_p_base += D_Zpinv_y;
      r_i_p_base += D_Zpinv_y_r;
      g_i_p_base += D_Zpinv_y_g;
      b_i_p_base += D_Zpinv_y_b;

      Vec2f s = P.getXY();
      F32 e_pq_s = n_pq.dot(s) + c_pq;
      if (e_pq_s >= 0) {
        F32 e_qr_s = n_qr.dot(s) + c_qr;
        if (e_qr_s >= 0) {
          F32 e_rp_s = n_rp.dot(s) + c_rp;
          if (e_rp_s < 0)
            continue;
        } else continue;
      } else continue;

      S32 idx = P.x + P.y * width;
      if (zbuffer[idx] < P.z) {
        zbuffer[idx] = P.z;
        put_pixel(pix, color, P.x, P.y);
      }
    }
  }
}

template<typename T>
class Device {
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
  struct ViewPort {
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

Mat4f viewport(S32 x, S32 y, S32 w, S32 h) {
  Mat4f m;
  m.setIdentity();
  m(0, 3) = x + w / 2.f;
  m(1, 3) = y + h / 2.f;
  m(2, 3) = depth / 2.f;

  m(0, 0) = w / 2.f;
  m(1, 1) = h / 2.f;
  m(2, 2) = depth / 2.f;
  return m;
}

Vec3f world2screen(Vec3f v) {
  return Vec3f(S32((v.x + 1.f) * width / 2.f + .5f), S32((v.y + 1.f) * height / 2.f + .5f), v.z);
}

Mat4f lookat(Vec3f eye, Vec3f center, Vec3f up) {
  Vec3f z = (eye - center).normalized();
  z.normalize();
  Vec3f x = cross(up,z);
  x.normalize();
  Vec3f y = z.cross(x);
  y.normalize();
  Mat4f res;
  res.setIdentity();
  for (S32 i = 0; i < 3; i++) {
    res(0, i) = x[i];
    res(1, i) = y[i];
    res(2, i) = z[i];
    res(i, 3) = -center[i];
  }
  return res;
}

S32 main(S32 argc, char **argv) {
//  if (2 == argc) {
//    model = new Model(argv[1]);
//  } else {
//#if defined __linux__ || defined __APPLE__
//    model = new Model("../../obj/african_head.obj");
//#else
//    model = new Model("../obj/african_head.obj");
//#endif //
//  }

  std::string inputfile = "../../obj/african_head.obj";
//  std::string inputfile = "../../obj/cottage_obj.obj";

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  std::string warn;
  std::string err;

  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputfile.c_str());

  if (!warn.empty()) {
    std::cout << warn << std::endl;
  }

  if (!err.empty()) {
    std::cerr << err << std::endl;
  }

  if (!ret) {
    exit(1);
  }

//add device
  Device<GLfloat> m_device;
  m_device.sr_glViewport(0, 0, 800, 800);
  m_device.sr_glDepthRange(0, 1);

  S32 *zbuffer = new S32[width * height];
  light_dir.normalize();
  void *pix;
  S32 pitch;

  init();

  S32 x_current(0);
  S32 y_current(0);

  // Main loop flag
  bool quit = false;

  // Event handler
  SDL_Event e;

  F32 deltaX(1), deltaY(1), deltaZ(3);

  eye = Vec3f( deltaX, deltaY, deltaZ);

  // timer
  clock_t start(0), finish(0);
  double duration;
  S32 frame_count(0);
  // While application is running
  while (!quit) {
    PROFILE_START_FRAME;
    PROFILE_BEGIN("FRAME");
    PROFILE_BEGIN("Prepare");
    frame_count++;
    if (frame_count == 60) {
      finish = clock();
      duration = (finish - start) / CLOCKS_PER_SEC;
      FW::printf("%f  FPS\n", 60 / duration);
      start = clock();
      frame_count = 0;
    }
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
    Clear((U32 *) pix, BLACK);

    // Eye and center position
    eye = Vec3f(deltaX, deltaY, deltaZ);

    Mat4f ModelView = lookat(eye, center, Vec3f(0, 1, 0));
    Mat4f Projection;
    Projection.setIdentity();
    Mat4f ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    Projection(3, 2) = -1.f / (eye - center).length();
    Mat4f z = ViewPort * Projection * ModelView;

    for (S32 i = 0; i < width * height; i++) {
      zbuffer[i] = std::numeric_limits<S32>::min();
    }
    PROFILE_END();
    PROFILE_BEGIN("Render face");
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {

      // Loop over faces(polygon)
      size_t index_offset = 0;
      for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
        Vec3f screen_coords[3];
        Vec3f world_coords[3];
        U32 colors[3];
        Vec3f n_v[3];
        F32 intensity[3];
        int fv = shapes[s].mesh.num_face_vertices[f];

        // Loop over vertices in the face.
        for (size_t v = 0; v < fv; v++) {
          // access to vertex
          tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
          tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
          tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
          tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];
          Vec3f vec(vx, vy, vz);
          tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
          tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
          tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
          n_v[v] = Vec3f(nx,ny,nz);
          tinyobj::real_t tx = attrib.texcoords[2*idx.texcoord_index+0];
          tinyobj::real_t ty = attrib.texcoords[2*idx.texcoord_index+1];
          // Optional: vertex colors
          // tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
          // tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
          // tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];
          Vec4f v_(vec, 1);
//        v_ << v, 1;
          Vec4f v_temp;
          v_temp = ViewPort * Projection * ModelView * v_;
          v_temp = v_temp / v_temp.w;
          screen_coords[v] = (v_temp).getXYZ();

          world_coords[v] = vec;
          intensity[v] = fabs(n_v[v].dot(light_dir));

#if  defined(WITH_TEXTURE)
          if (intensity[j] > 0) {
          colors[j] = SDL_MapRGBA(pixFormat,
                                  static_cast<U8>(model->diffuse(model->uv(i, j))[2] * intensity[j]),
                                  static_cast<U8>(model->diffuse(model->uv(i, j))[1] * intensity[j]),
                                  static_cast<U8>(model->diffuse(model->uv(i, j))[0] * intensity[j]),
                                  model->diffuse(model->uv(i, j))[3]);
        } else {
          colors[j] = SDL_MapRGBA(pixFormat,
                                  model->diffuse(model->uv(i, j))[2],
                                  model->diffuse(model->uv(i, j))[1],
                                  model->diffuse(model->uv(i, j))[0],
                                  model->diffuse(model->uv(i, j))[3]);
        }
#else
          if (intensity[v] > 0) {
            colors[v] = SDL_MapRGBA(pixFormat,
                                    static_cast<U8>(255.0 * intensity[v]),
                                    static_cast<U8>(255.0 * intensity[v]),
                                    static_cast<U8>(255.0 * intensity[v]),
                                    255);
          }
#endif

        }
        index_offset += fv;

        // per-face material
        shapes[s].mesh.material_ids[f];
//      }
//    }

        // Render face
//    for (S32 i = 0; i < model->nfaces(); i++) {
//      std::vector<S32> face = model->face(i);
//      Vec3f screen_coords[3];
//      Vec3f world_coords[3];
//      U32 colors[3];
//      Vec3f n_v[3];
//      F32 intensity[3];
//
//      // load texture data
//      for (S32 j = 0; j < 3; ++j) {
//        Vec3f v = model->vert(face[j]);
//        n_v[j] = model->normal(i, j);
//
//        Vec4f v_(v,1);
////        v_ << v, 1;
//        Vec4f v_temp;
//        v_temp = ViewPort * Projection * ModelView * v_;
//        v_temp = v_temp / v_temp.w;
//        screen_coords[j] = (v_temp).getXYZ();
//
//        world_coords[j] = v;
//        intensity[j] = fabs(n_v[j].dot(light_dir));
//#if  defined(WITH_TEXTURE)
//        if (intensity[j] > 0) {
//          colors[j] = SDL_MapRGBA(pixFormat,
//                                  static_cast<U8>(model->diffuse(model->uv(i, j))[2] * intensity[j]),
//                                  static_cast<U8>(model->diffuse(model->uv(i, j))[1] * intensity[j]),
//                                  static_cast<U8>(model->diffuse(model->uv(i, j))[0] * intensity[j]),
//                                  model->diffuse(model->uv(i, j))[3]);
//        } else {
//          colors[j] = SDL_MapRGBA(pixFormat,
//                                  model->diffuse(model->uv(i, j))[2],
//                                  model->diffuse(model->uv(i, j))[1],
//                                  model->diffuse(model->uv(i, j))[0],
//                                  model->diffuse(model->uv(i, j))[3]);
//        }
//#else
//		if (intensity[j] > 0) {
//			colors[j] = SDL_MapRGBA(pixFormat,
//				static_cast<U8>(255.0 * intensity[j]),
//				static_cast<U8>(255.0 * intensity[j]),
//				static_cast<U8>(255.0 * intensity[j]),
//				255);
//		}
//#endif
//
//      }


        // back face culling
        Vec3f AB = screen_coords[0] - screen_coords[1];
        Vec3f AC = screen_coords[0] - screen_coords[2];
        Vec3f N = AC.cross(AB);
        if (N.z > 0) {
          continue;
        }

        // tessellation
//      Vec3i screen_coords_i[3];
//      screen_coords_i[0] = screen_coords[0];
//      screen_coords_i[1] = screen_coords[1];
//      screen_coords_i[2] = screen_coords[2];
        TriangleFast3(screen_coords, zbuffer, (U32 *) pix, colors);
      }
    }
    PROFILE_END();
//    Vec3f world_coords[3];
//    world_coords[0] << 0, 0, 0.99;
//    world_coords[1] << 1, 0.5, 0.99;
//    world_coords[2] << 0.5, 1, -0.99;
//
//    Vec3f screen_coords[3];
//
//    U32 colors[3];
//    colors[0] = RED;
//    colors[1] = GREEN;
//    colors[2] = BLUE;
//    for (S32 j = 0; j < 3; ++j) {
//      Vec4f v_;
//      v_ << world_coords[j], 1;
//      Vec4f v_temp;
//      v_temp = ViewPort * Projection * ModelView * v_;
//      v_temp = v_temp / v_temp.w;
//      screen_coords[j] = v_temp.block(0, 0, 3, 1);
//    }

    // tessellation
//    TriangleFast2(screen_coords, zbuffer, (U32 *) pix, colors);
    PROFILE_BEGIN("Render present");
    // Render end
    SDL_UnlockTexture(gTexture);
    SDL_RenderCopy(gRender, gTexture, NULL, NULL);
    SDL_RenderPresent(gRender);
    PROFILE_END();
    PROFILE_END();
    cout << ofxProfiler::getResults();

#if defined __linux__ || defined __APPLE__
    usleep(10);
#else
    Sleep(1);
#endif //

  }

//  { // dump z-buffer (debugging purposes only)
//    TGAImage zbimage(width, height, TGAImage::GRAYSCALE);
//    for (S32 i = 0; i < width; i++) {
//      for (S32 j = 0; j < height; j++) {
//        unsigned char zzzz = zbuffer[i + j * width];
//        zbimage.set(i, j, TGAColor(&zzzz, 1));
//      }
//    }
//    zbimage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
//    zbimage.write_tga_file("zbuffer.tga");
//  }

  close();
  return 0;
}
