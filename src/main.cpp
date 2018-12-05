//
// Created by Macx on 2018/10/6.
//

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <zconf.h>
#include "../imported/eigen/Eigen/Dense"
#include "../imported/eigen/Eigen/Geometry"

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793f
#define INFINITY 1e8
#endif

#include <SDL2/SDL.h>
#include <cmath>
#include <iostream>
const int width = 640;
const int height = 480;

using Eigen::Vector3f;
using Eigen::Matrix4f;

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
  pix[y * width + x] = color;
}

int main() {

  void *pix;
  int pitch;
  SDL_PixelFormat *format;
  init();
  format = SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888);

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
      if (e.type == SDL_QUIT) {
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

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        Uint32 clor = SDL_MapRGBA(SDL_AllocFormat(SDL_PIXELFORMAT_RGBA8888), 0xFF>>1, 0XFF>>2, 0XFF>>3, 0XFF);
        put_pixel((Uint32 *) pix, clor, j, i);
      }
    }
    SDL_UnlockTexture(gTexture);

    SDL_RenderClear(gRender);
    SDL_RenderCopy(gRender, gTexture, NULL, NULL);
    SDL_RenderPresent(gRender);
  }
  close();
}
