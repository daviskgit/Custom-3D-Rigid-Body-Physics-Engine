// renderer.h - a tiny Win32 + OpenGL 4.5 core-profile renderer. Owns the window,
// the GL context, and an instanced unit-cube draw call used for every body.
#pragma once
#include <vector>
#include "gl_core.h"
#include "math.h"

namespace pe {

struct InstanceData {
    Mat4 model;
    Vec3 color;
    Real _pad = 0;
};

struct Input {
    bool keys[256] = {};
    int mouseDX = 0, mouseDY = 0;
    int wheelDelta = 0;
    bool mouseDown = false;
    bool quit = false;
};

class Renderer {
public:
    bool init(int width, int height, const char* title);
    void shutdown();

    bool pumpEvents(Input& input); // returns false when the window should close
    void beginFrame(const Vec3& clearColor);
    void drawInstances(const std::vector<InstanceData>& instances);
    void endFrame();

    void setCamera(const Mat4& view, const Mat4& proj) { view_ = view; proj_ = proj; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    HWND hwnd_ = nullptr;
    HDC hdc_ = nullptr;
    HGLRC hglrc_ = nullptr;
    int width_ = 0, height_ = 0;

    GLuint program_ = 0;
    GLuint cubeVAO_ = 0, cubeVBO_ = 0, cubeEBO_ = 0, instanceVBO_ = 0;
    GLint locView_ = -1, locProj_ = -1, locLightDir_ = -1;
    Mat4 view_ = Mat4::identity(), proj_ = Mat4::identity();

    void buildShaders();
    void buildCubeMesh();
};

} // namespace pe
