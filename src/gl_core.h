// gl_core.h - just enough of the OpenGL 4.5 core API, declared by hand so the
// engine has zero third-party dependencies (no GLFW/GLAD/GLEW). Windows exports
// GL 1.1 directly from opengl32.dll; everything newer is fetched at runtime via
// wglGetProcAddress by loadGLFunctions() in renderer.cpp.
#pragma once
#include <cstddef>
#include <cstdint>
#include <windows.h>
#include <GL/gl.h>

typedef char GLchar;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

#define GL_ARRAY_BUFFER 0x8892
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_STATIC_DRAW 0x88E4
#define GL_DYNAMIC_DRAW 0x88E8
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_VERTEX_SHADER 0x8B31
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#define GL_INFO_LOG_LENGTH 0x8B84
#define GL_MULTISAMPLE 0x809D
#define GL_DEBUG_OUTPUT 0x92E0
#define GL_DEBUG_OUTPUT_SYNCHRONOUS 0x8242
#define GL_MAJOR_VERSION 0x821B
#define GL_MINOR_VERSION 0x821C

// ---- WGL extension tokens (windows.h only ships WGL 1.0) --------------------
#define WGL_CONTEXT_MAJOR_VERSION_ARB 0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB 0x2092
#define WGL_CONTEXT_PROFILE_MASK_ARB 0x9126
#define WGL_CONTEXT_FLAGS_ARB 0x2094
#define WGL_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001
#define WGL_DRAW_TO_WINDOW_ARB 0x2001
#define WGL_SUPPORT_OPENGL_ARB 0x2010
#define WGL_DOUBLE_BUFFER_ARB 0x2011
#define WGL_PIXEL_TYPE_ARB 0x2013
#define WGL_TYPE_RGBA_ARB 0x202B
#define WGL_COLOR_BITS_ARB 0x2014
#define WGL_DEPTH_BITS_ARB 0x2022
#define WGL_STENCIL_BITS_ARB 0x2023
#define WGL_ACCELERATION_ARB 0x2003
#define WGL_FULL_ACCELERATION_ARB 0x2027
#define WGL_SAMPLE_BUFFERS_ARB 0x2041
#define WGL_SAMPLES_ARB 0x2042

typedef HGLRC(WINAPI* PFNWGLCREATECONTEXTATTRIBSARBPROC)(HDC, HGLRC, const int*);
typedef BOOL(WINAPI* PFNWGLCHOOSEPIXELFORMATARBPROC)(HDC, const int*, const FLOAT*, UINT, int*, UINT*);
typedef BOOL(WINAPI* PFNWGLSWAPINTERVALEXTPROC)(int);

// ---- GL >= 1.5 entry points loaded dynamically -------------------------------
typedef void(APIENTRY* PFNGLGENBUFFERSPROC)(GLsizei, GLuint*);
typedef void(APIENTRY* PFNGLBINDBUFFERPROC)(GLenum, GLuint);
typedef void(APIENTRY* PFNGLBUFFERDATAPROC)(GLenum, GLsizeiptr, const void*, GLenum);
typedef void(APIENTRY* PFNGLBUFFERSUBDATAPROC)(GLenum, GLintptr, GLsizeiptr, const void*);
typedef void(APIENTRY* PFNGLGENVERTEXARRAYSPROC)(GLsizei, GLuint*);
typedef void(APIENTRY* PFNGLBINDVERTEXARRAYPROC)(GLuint);
typedef void(APIENTRY* PFNGLENABLEVERTEXATTRIBARRAYPROC)(GLuint);
typedef void(APIENTRY* PFNGLVERTEXATTRIBPOINTERPROC)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
typedef void(APIENTRY* PFNGLVERTEXATTRIBDIVISORPROC)(GLuint, GLuint);
typedef GLuint(APIENTRY* PFNGLCREATESHADERPROC)(GLenum);
typedef void(APIENTRY* PFNGLSHADERSOURCEPROC)(GLuint, GLsizei, const GLchar* const*, const GLint*);
typedef void(APIENTRY* PFNGLCOMPILESHADERPROC)(GLuint);
typedef void(APIENTRY* PFNGLGETSHADERIVPROC)(GLuint, GLenum, GLint*);
typedef void(APIENTRY* PFNGLGETSHADERINFOLOGPROC)(GLuint, GLsizei, GLsizei*, GLchar*);
typedef GLuint(APIENTRY* PFNGLCREATEPROGRAMPROC)(void);
typedef void(APIENTRY* PFNGLATTACHSHADERPROC)(GLuint, GLuint);
typedef void(APIENTRY* PFNGLLINKPROGRAMPROC)(GLuint);
typedef void(APIENTRY* PFNGLGETPROGRAMIVPROC)(GLuint, GLenum, GLint*);
typedef void(APIENTRY* PFNGLGETPROGRAMINFOLOGPROC)(GLuint, GLsizei, GLsizei*, GLchar*);
typedef void(APIENTRY* PFNGLUSEPROGRAMPROC)(GLuint);
typedef void(APIENTRY* PFNGLDELETESHADERPROC)(GLuint);
typedef GLint(APIENTRY* PFNGLGETUNIFORMLOCATIONPROC)(GLuint, const GLchar*);
typedef void(APIENTRY* PFNGLUNIFORMMATRIX4FVPROC)(GLint, GLsizei, GLboolean, const GLfloat*);
typedef void(APIENTRY* PFNGLUNIFORM3FVPROC)(GLint, GLsizei, const GLfloat*);
typedef void(APIENTRY* PFNGLUNIFORM1FPROC)(GLint, GLfloat);
typedef void(APIENTRY* PFNGLUNIFORM1IPROC)(GLint, GLint);
typedef void(APIENTRY* PFNGLDRAWARRAYSINSTANCEDPROC)(GLenum, GLint, GLsizei, GLsizei);
typedef void(APIENTRY* PFNGLDRAWELEMENTSINSTANCEDPROC)(GLenum, GLsizei, GLenum, const void*, GLsizei);
typedef void(APIENTRY* PFNGLDELETEBUFFERSPROC)(GLsizei, const GLuint*);
typedef void(APIENTRY* PFNGLDELETEVERTEXARRAYSPROC)(GLsizei, const GLuint*);
typedef void(APIENTRY* PFNGLDELETEPROGRAMPROC)(GLuint);

#define GL_FUNC_LIST \
    X(PFNGLGENBUFFERSPROC, glGenBuffers) \
    X(PFNGLBINDBUFFERPROC, glBindBuffer) \
    X(PFNGLBUFFERDATAPROC, glBufferData) \
    X(PFNGLBUFFERSUBDATAPROC, glBufferSubData) \
    X(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays) \
    X(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray) \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray) \
    X(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer) \
    X(PFNGLVERTEXATTRIBDIVISORPROC, glVertexAttribDivisor) \
    X(PFNGLCREATESHADERPROC, glCreateShader) \
    X(PFNGLSHADERSOURCEPROC, glShaderSource) \
    X(PFNGLCOMPILESHADERPROC, glCompileShader) \
    X(PFNGLGETSHADERIVPROC, glGetShaderiv) \
    X(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog) \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram) \
    X(PFNGLATTACHSHADERPROC, glAttachShader) \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram) \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv) \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog) \
    X(PFNGLUSEPROGRAMPROC, glUseProgram) \
    X(PFNGLDELETESHADERPROC, glDeleteShader) \
    X(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation) \
    X(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv) \
    X(PFNGLUNIFORM3FVPROC, glUniform3fv) \
    X(PFNGLUNIFORM1FPROC, glUniform1f) \
    X(PFNGLUNIFORM1IPROC, glUniform1i) \
    X(PFNGLDRAWARRAYSINSTANCEDPROC, glDrawArraysInstanced) \
    X(PFNGLDRAWELEMENTSINSTANCEDPROC, glDrawElementsInstanced) \
    X(PFNGLDELETEBUFFERSPROC, glDeleteBuffers) \
    X(PFNGLDELETEVERTEXARRAYSPROC, glDeleteVertexArrays) \
    X(PFNGLDELETEPROGRAMPROC, glDeleteProgram)

#define X(type, name) extern type name;
GL_FUNC_LIST
#undef X

bool loadGLFunctions(); // call once after the context is current
