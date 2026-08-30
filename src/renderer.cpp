#include "renderer.h"
#include <cstdio>
#include <cstring>

#define X(type, name) type name = nullptr;
GL_FUNC_LIST
#undef X

static PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB_ = nullptr;
static PFNWGLCHOOSEPIXELFORMATARBPROC wglChoosePixelFormatARB_ = nullptr;
static PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT_ = nullptr;

bool loadGLFunctions() {
    bool ok = true;
#define X(type, name)                                                   \
    name = (type)wglGetProcAddress(#name);                              \
    if (!name) { fprintf(stderr, "missing GL entry point: %s\n", #name); ok = false; }
    GL_FUNC_LIST
#undef X
    return ok;
}

namespace pe {

static Input* g_input = nullptr;

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_CLOSE:
        case WM_DESTROY:
            // Only the real window (tracked via g_input, wired up once pumpEvents
            // starts) should terminate the loop; the throwaway dummy window used
            // to fetch WGL_ARB entry points is destroyed before that happens.
            if (g_input) g_input->quit = true;
            return 0;
        case WM_KEYDOWN:
            if (g_input && wp < 256) g_input->keys[wp] = true;
            return 0;
        case WM_KEYUP:
            if (g_input && wp < 256) g_input->keys[wp] = false;
            return 0;
        case WM_MOUSEWHEEL:
            if (g_input) g_input->wheelDelta += GET_WHEEL_DELTA_WPARAM(wp);
            return 0;
        case WM_LBUTTONDOWN: if (g_input) g_input->mouseDown = true; SetCapture(hwnd); return 0;
        case WM_LBUTTONUP: if (g_input) g_input->mouseDown = false; ReleaseCapture(); return 0;
    }
    return DefWindowProcA(hwnd, msg, wp, lp);
}

bool Renderer::init(int width, int height, const char* title) {
    width_ = width; height_ = height;
    HINSTANCE hInst = GetModuleHandleA(nullptr);

    WNDCLASSA wc{};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = "PhysicsEngineWindowClass";
    wc.hCursor = LoadCursorA(nullptr, IDC_ARROW);
    RegisterClassA(&wc);

    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.cStencilBits = 8;

    // --- Step 1: a throwaway hidden window + legacy context, used only to
    // fetch the WGL_ARB entry points. Its WM_DESTROY must not reach the real
    // message loop (WndProc's WM_DESTROY posts WM_QUIT), so it never becomes hwnd_.
    HWND dummy = CreateWindowExA(0, wc.lpszClassName, "dummy", WS_OVERLAPPEDWINDOW,
                                 0, 0, 1, 1, nullptr, nullptr, hInst, nullptr);
    HDC dummyDC = GetDC(dummy);
    int pf = ChoosePixelFormat(dummyDC, &pfd);
    if (!pf || !SetPixelFormat(dummyDC, pf, &pfd)) return false;
    HGLRC tempCtx = wglCreateContext(dummyDC);
    if (!tempCtx) return false;
    wglMakeCurrent(dummyDC, tempCtx);

    wglCreateContextAttribsARB_ = (PFNWGLCREATECONTEXTATTRIBSARBPROC)wglGetProcAddress("wglCreateContextAttribsARB");
    wglChoosePixelFormatARB_ = (PFNWGLCHOOSEPIXELFORMATARBPROC)wglGetProcAddress("wglChoosePixelFormatARB");
    wglSwapIntervalEXT_ = (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");

    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(tempCtx);
    ReleaseDC(dummy, dummyDC);
    DestroyWindow(dummy); // fine here: g_input is still null, so WM_DESTROY is a no-op for us

    if (!wglCreateContextAttribsARB_) { fprintf(stderr, "WGL_ARB_create_context unavailable\n"); return false; }

    // --- Step 2: create the real, visible window exactly once, with the best
    // pixel format (MSAA if available) applied before any context is attached.
    RECT r{0, 0, width, height};
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    hwnd_ = CreateWindowExA(0, wc.lpszClassName, title, WS_OVERLAPPEDWINDOW,
                            CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left, r.bottom - r.top,
                            nullptr, nullptr, hInst, nullptr);
    if (!hwnd_) return false;
    hdc_ = GetDC(hwnd_);

    int samples = 4;
    int pfAttribs[] = {
        WGL_DRAW_TO_WINDOW_ARB, TRUE, WGL_SUPPORT_OPENGL_ARB, TRUE, WGL_DOUBLE_BUFFER_ARB, TRUE,
        WGL_PIXEL_TYPE_ARB, WGL_TYPE_RGBA_ARB, WGL_COLOR_BITS_ARB, 32, WGL_DEPTH_BITS_ARB, 24,
        WGL_STENCIL_BITS_ARB, 8, WGL_ACCELERATION_ARB, WGL_FULL_ACCELERATION_ARB,
        WGL_SAMPLE_BUFFERS_ARB, 1, WGL_SAMPLES_ARB, samples, 0,
    };
    UINT numFormats = 0;
    int chosenPF = 0;
    if (!wglChoosePixelFormatARB_ || !wglChoosePixelFormatARB_(hdc_, pfAttribs, nullptr, 1, &chosenPF, &numFormats) || numFormats == 0) {
        chosenPF = ChoosePixelFormat(hdc_, &pfd); // MSAA/ARB path unavailable; fall back
    }
    if (!SetPixelFormat(hdc_, chosenPF, &pfd)) return false;

    int ctxAttribs[] = {
        WGL_CONTEXT_MAJOR_VERSION_ARB, 4, WGL_CONTEXT_MINOR_VERSION_ARB, 5,
        WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB, 0,
    };
    hglrc_ = wglCreateContextAttribsARB_(hdc_, nullptr, ctxAttribs);
    if (!hglrc_) {
        // Some drivers cap out below 4.5; fall back to whatever core profile they offer.
        int fallback[] = {WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB, 0};
        hglrc_ = wglCreateContextAttribsARB_(hdc_, nullptr, fallback);
    }
    if (!hglrc_) return false;
    wglMakeCurrent(hdc_, hglrc_);

    if (!loadGLFunctions()) return false;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    if (wglSwapIntervalEXT_) wglSwapIntervalEXT_(1); // vsync

    ShowWindow(hwnd_, SW_SHOW);
    SetForegroundWindow(hwnd_);
    SetFocus(hwnd_);

    buildShaders();
    buildCubeMesh();
    return true;
}

void Renderer::shutdown() {
    if (program_) glDeleteProgram(program_);
    GLuint bufs[] = {cubeVBO_, cubeEBO_, instanceVBO_};
    glDeleteBuffers(3, bufs);
    glDeleteVertexArrays(1, &cubeVAO_);
    if (hglrc_) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(hglrc_); }
    if (hdc_) ReleaseDC(hwnd_, hdc_);
    if (hwnd_) DestroyWindow(hwnd_);
}

bool Renderer::pumpEvents(Input& input) {
    g_input = &input;
    input.mouseDX = input.mouseDY = 0;
    input.wheelDelta = 0;

    static POINT lastPt{-1, -1};
    POINT cur; GetCursorPos(&cur);
    if (input.mouseDown && lastPt.x != -1) {
        input.mouseDX = cur.x - lastPt.x;
        input.mouseDY = cur.y - lastPt.y;
    }
    lastPt = cur;

    MSG msg;
    while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) input.quit = true;
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    RECT rc; GetClientRect(hwnd_, &rc);
    width_ = rc.right - rc.left; height_ = rc.bottom - rc.top;
    return !input.quit;
}

void Renderer::beginFrame(const Vec3& clearColor) {
    glViewport(0, 0, width_, height_);
    glClearColor(clearColor.x, clearColor.y, clearColor.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(program_);
    glUniformMatrix4fv(locView_, 1, GL_FALSE, view_.m);
    glUniformMatrix4fv(locProj_, 1, GL_FALSE, proj_.m);
    Vec3 lightDir = normalize(Vec3{0.4f, 1.0f, 0.3f});
    glUniform3fv(locLightDir_, 1, &lightDir.x);
}

void Renderer::drawInstances(const std::vector<InstanceData>& instances) {
    if (instances.empty()) return;
    glBindVertexArray(cubeVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(instances.size() * sizeof(InstanceData)),
                instances.data(), GL_DYNAMIC_DRAW);
    glDrawElementsInstanced(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr, (GLsizei)instances.size());
}

void Renderer::endFrame() { SwapBuffers(hdc_); }

// ---------------------------------------------------------------------------
static GLuint compileShader(GLenum type, const char* src) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = 0;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        GLsizei n = 0;
        glGetShaderInfoLog(sh, sizeof(log), &n, log);
        fprintf(stderr, "shader compile error: %.*s\n", n, log);
    }
    return sh;
}

void Renderer::buildShaders() {
    static const char* vs = R"GLSL(
#version 450 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in mat4 aModel;   // consumes attrib locations 2,3,4,5
layout(location = 6) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProj;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorldPos;

void main() {
    vec4 world = aModel * vec4(aPos, 1.0);
    vWorldPos = world.xyz;
    vNormal = mat3(aModel) * aNormal;
    vColor = aColor;
    gl_Position = uProj * uView * world;
}
)GLSL";

    static const char* fs = R"GLSL(
#version 450 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorldPos;
uniform vec3 uLightDir;
out vec4 FragColor;

void main() {
    vec3 n = normalize(vNormal);
    float diff = max(dot(n, normalize(uLightDir)), 0.0);
    float ambient = 0.28;
    vec3 color = vColor * (ambient + diff * 0.72);
    // gentle distance fog so stacked scenes read as 3D
    float fog = clamp(length(vWorldPos) / 60.0, 0.0, 0.35);
    color = mix(color, vec3(0.53, 0.6, 0.66), fog);
    FragColor = vec4(color, 1.0);
}
)GLSL";

    GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    program_ = glCreateProgram();
    glAttachShader(program_, v);
    glAttachShader(program_, f);
    glLinkProgram(program_);
    GLint ok = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048]; GLsizei n = 0;
        glGetProgramInfoLog(program_, sizeof(log), &n, log);
        fprintf(stderr, "program link error: %.*s\n", n, log);
    }
    glDeleteShader(v);
    glDeleteShader(f);

    locView_ = glGetUniformLocation(program_, "uView");
    locProj_ = glGetUniformLocation(program_, "uProj");
    locLightDir_ = glGetUniformLocation(program_, "uLightDir");
}

void Renderer::buildCubeMesh() {
    // position + normal, 24 verts (4 per face) so normals are flat-shaded.
    struct V { float px, py, pz, nx, ny, nz; };
    static const V verts[] = {
        // -Z
        {-1,-1,-1, 0,0,-1}, {-1,1,-1, 0,0,-1}, {1,1,-1, 0,0,-1}, {1,-1,-1, 0,0,-1},
        // +Z
        {-1,-1,1, 0,0,1}, {1,-1,1, 0,0,1}, {1,1,1, 0,0,1}, {-1,1,1, 0,0,1},
        // -X
        {-1,-1,-1, -1,0,0}, {-1,-1,1, -1,0,0}, {-1,1,1, -1,0,0}, {-1,1,-1, -1,0,0},
        // +X
        {1,-1,-1, 1,0,0}, {1,1,-1, 1,0,0}, {1,1,1, 1,0,0}, {1,-1,1, 1,0,0},
        // -Y
        {-1,-1,-1, 0,-1,0}, {1,-1,-1, 0,-1,0}, {1,-1,1, 0,-1,0}, {-1,-1,1, 0,-1,0},
        // +Y
        {-1,1,-1, 0,1,0}, {-1,1,1, 0,1,0}, {1,1,1, 0,1,0}, {1,1,-1, 0,1,0},
    };
    static const unsigned idx[] = {
        0,1,2, 0,2,3,       4,5,6, 4,6,7,       8,9,10, 8,10,11,
        12,13,14, 12,14,15, 16,17,18, 16,18,19, 20,21,22, 20,22,23,
    };

    glGenVertexArrays(1, &cubeVAO_);
    glBindVertexArray(cubeVAO_);

    glGenBuffers(1, &cubeVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(V), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(V), (void*)(3 * sizeof(float)));

    glGenBuffers(1, &cubeEBO_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, GL_STATIC_DRAW);

    // Per-instance model matrix (4 vec4 columns, locations 2-5) + color (location 6).
    glGenBuffers(1, &instanceVBO_);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO_);
    for (int i = 0; i < 4; ++i) {
        glEnableVertexAttribArray(2 + i);
        glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                              (void*)(size_t)(i * 4 * sizeof(float)));
        glVertexAttribDivisor(2 + i, 1);
    }
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void*)offsetof(InstanceData, color));
    glVertexAttribDivisor(6, 1);
}

} // namespace pe
