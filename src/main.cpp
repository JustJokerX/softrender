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

const int width = 640;
const int height = 480;

using Eigen::Vector3f;
using Eigen::Matrix4f;

Uint32 WHITE = SDL_MapRGBA(SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888), 0xFF, 0xFF, 0xFF, 0xFF);
Uint32 BLACK = SDL_MapRGBA(SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888), 0x00, 0x00, 0x00, 0xFF);
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
  pix[(height-y-1) * width + x] = color;
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
int main(int argc, char** argv) {
  if (2==argc) {
    model = new Model(argv[1]);
  } else {
    model = new Model("../../obj/african_head.obj");
  }

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
    Clear((Uint32 *) pix, BLACK);
//    Line(0, 0, width, height, (Uint32 *) pix, WHITE);
    for (int i=0; i<model->nfaces(); i++) {
      std::vector<int> face = model->face(i);
      for (int j=0; j<3; j++) {
        Vector3f v0 = model->vert(face[j]);
        Vector3f v1 = model->vert(face[(j+1)%3]);
        int x0 = (v0.x()+1.)*width/2.;
        int y0 = (v0.y()+1.)*height/2.;
        int x1 = (v1.x()+1.)*width/2.;
        int y1 = (v1.y()+1.)*height/2.;
        Line(x0, y0, x1, y1, (Uint32 *)pix, WHITE);
      }
    }
    SDL_UnlockTexture(gTexture);

    SDL_RenderCopy(gRender, gTexture, NULL, NULL);
    SDL_RenderPresent(gRender);
  }
  close();
}
