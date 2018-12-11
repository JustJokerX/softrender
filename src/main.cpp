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

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793f
#define INFINITY 1e8
#endif

#include <SDL2/SDL.h>
#include <zconf.h>

const int width = 800;
const int height = 800;

using Eigen::Vector3f;
using Eigen::Matrix4f;

SDL_PixelFormat *pixFormat = SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888);

Uint32 WHITE = SDL_MapRGBA(pixFormat, 255, 255, 255, 255);
Uint32 BLACK = SDL_MapRGBA(pixFormat, 0, 0, 0, 255);
Uint32 RED = SDL_MapRGBA(pixFormat, 255, 0, 0, 255);
Uint32 GREEN = SDL_MapRGBA(pixFormat, 0, 255, 0, 255);

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

Vector3f barycentric(Vector3f A, Vector3f B, Vector3f C, Vector3f P) {
  Vector3f s[2];
  for (int i = 2; i--;) {
    s[i][0] = C[i] - A[i];
    s[i][1] = B[i] - A[i];
    s[i][2] = A[i] - P[i];
  }
  Vector3f u = s[0].cross(s[1]);
  if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
    return Vector3f(1.f - (u.x() + u.y()) / u.z(), u.y() / u.z(), u.x() / u.z());
  return Vector3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

void Triangle(Vector3f *pts, float *zbuffer, Uint32 *pix, Uint32 color) {
  Vector2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  Vector2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
  Vector2f clamp(width - 1, height - 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts[i][j]));
      bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
    }
  }
  Vector3f P;
  for (P.x() = bboxmin.x(); P.x() <= bboxmax.x(); P.x()++) {
    for (P.y() = bboxmin.y(); P.y() <= bboxmax.y(); P.y()++) {
      Vector3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
      if (bc_screen.x() < 0 || bc_screen.y() < 0 || bc_screen.z() < 0) continue;
      P.z() = 0;
      for (int i = 0; i < 3; i++) P.z() += pts[i][2] * bc_screen[i];
      if (zbuffer[int(P.x() + P.y() * width)] < P.z()) {
        zbuffer[int(P.x() + P.y() * width)] = P.z();
        put_pixel(pix, color, P.x(), P.y());
      }
    }
  }
}

void Triangle(Vector3f *pts, float *zbuffer, Uint32 *pix, Uint32 *colors) {
  Vector2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  Vector2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
  Vector2f clamp(width - 1, height - 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts[i][j]));
      bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
    }
  }
  Vector3f P;
  for (P.x() = bboxmin.x(); P.x() <= bboxmax.x(); P.x()++) {
    for (P.y() = bboxmin.y(); P.y() <= bboxmax.y(); P.y()++) {
      Vector3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
      if (bc_screen.x() < 0 || bc_screen.y() < 0 || bc_screen.z() < 0) continue;
      P.z() = 0;
      for (int i = 0; i < 3; i++) P.z() += pts[i][2] * bc_screen[i];
      Uint8 r_add = 0, g_add = 0, b_add = 0, a_add = 0;
      {
        Uint8 r, g, b, a;
        for (int i = 0; i < 3; i++) {
          SDL_GetRGBA(colors[i], pixFormat, &r, &g, &b, &a);
          r_add += r * bc_screen[i];
          g_add += g * bc_screen[i];
          b_add += b * bc_screen[i];
          a_add += a * bc_screen[i];
        }
      }

      Uint32 color = SDL_MapRGBA(pixFormat, r_add, g_add, b_add, a_add);

      if (zbuffer[int(P.x() + P.y() * width)] < P.z()) {
        zbuffer[int(P.x() + P.y() * width)] = P.z();
        put_pixel(pix, color, P.x(), P.y());
      }
    }
  }
}

Vector3f world2screen(Vector3f v) {
  return Vector3f(int((v.x() + 1.) * width / 2. + .5), int((v.y() + 1.) * height / 2. + .5), v.z());
}

int main(int argc, char **argv) {
  if (2 == argc) {
    model = new Model(argv[1]);
  } else {
    model = new Model("../../obj/african_head.obj");
  }

  Vector3f light_dir(0, 0, -1.);

  float *zbuffer = new float[width * height];

  void *pix;
  int pitch;

  init();

  int x_current(0);
  int y_current(0);

  // Main loop flag
  bool quit = false;

  // Event handler
  SDL_Event e;

  float deltaX(0), deltaY(0), deltaZ(0);

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

    for (int i = width * height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    // Render face
    for (int i = 0; i < model->nfaces(); i++) {
      std::vector<int> face = model->face(i);
      Vector3f pts[3];
      for (int i = 0; i < 3; i++) pts[i] = world2screen(model->vert(face[i]));
      Uint32 colors[3];
      for (int j = 0; j < 3; ++j) {
        colors[j] = SDL_MapRGBA(pixFormat, rand() % 255, rand() % 255, rand() % 255,
                                255);
      }
      Triangle(pts, zbuffer, (Uint32 *) pix, colors);
    }

    // Render end
    SDL_UnlockTexture(gTexture);
    SDL_RenderCopy(gRender, gTexture, NULL, NULL);
    SDL_RenderPresent(gRender);
    usleep(1000);
  }
  close();
}
