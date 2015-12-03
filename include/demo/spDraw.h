
#ifndef SP_DRAW_H
#define SP_DRAW_H

#include "spDemoPlatform.h"
#include "spring\spring.h"
#include "demo\spShader.h"
#include <GL\glew.h>

typedef struct { GLfloat x, y; } spVec;
typedef struct { GLfloat x, y, z; } spBary;
typedef struct _spColor { GLfloat r, g, b, a; } spColor;
typedef struct _spVertex { spVec pos; spVec aliasing; spBary barycentric; spColor fill; spColor outline; } spVertex;
typedef struct { spVertex a, b, c; } spTriangle;
typedef struct { spInt width, height; } spViewport;
typedef struct { spFloat left, right, top, bottom, near, far; } spFrustum;

#define spFrustumUniform(s) (spFrustum){-s, s, s, -s, -s, s }
#define spFrustumView(w, h) (spFrustum){-w, w, h, -h, -w, w}
#define spViewportNew(w, h) (spViewport){w, h}

/// some useful colors
#define RGBA255(r, g, b, a) ((spColor){(GLfloat)r/255.f, (GLfloat)g/255.f, (GLfloat)b/255.f, (GLfloat)a/255.f })
#define RGB255(r, g, b)     ((spColor){(GLfloat)r/255.f, (GLfloat)g/255.f, (GLfloat)b/255.f, (GLfloat)1 })
#define RGBA(r, g, b, a)    ((spColor){(GLfloat)r, (GLfloat)g, (GLfloat)b, (GLfloat)a })
#define RGB(r, g, b)        ((spColor){(GLfloat)r, (GLfloat)g, (GLfloat)b, (GLfloat)1 })
#define COLA(c, a)          ((spColor){(GLfloat)c, (GLfloat)c, (GLfloat)c, (GLfloat)a })

#define RED()    RGBA(0.5f, 0.0f, 0.0f, 1.0) 
#define BLUE()   RGBA(0.0f, 0.0f, 0.5f, 1.0) 
#define GREEN()  RGBA(0.0f, 0.5f, 0.0f, 1.0) 
#define YELLOW() RGBA(0.5f, 0.5f, 0.0f, 1.0) 
#define PURPLE() RGBA(0.5f, 0.0f, 0.5f, 1.0) 
#define BLACK()  COLA(0.0f, 1.0) 
#define WHITE()  COLA(1.0f, 1.0) 

typedef struct 
{
    spTriangle* buffer;
    GLuint vertexArray, vertexBuffer;
    GLint vertexShader, pixelShader, shaderProgram;
    GLint triangles, capacity;
} spRenderContext;

DEMO_API spVector spDeproject(spVector position, const spFloat model[16], const spFloat proj[16], spViewport view);
DEMO_API void spOrthoMatrix(spFloat ortho[16]);
DEMO_API void spDrawInit();
DEMO_API void spDrawLine(spVector start, spVector end, spFloat size, spColor color, spColor border);
DEMO_API void spDrawPolygon(spVector position, spFloat angle, spVector* verts, spInt count, spVector center, spColor color, spColor border);
DEMO_API void spDrawSegment(spVector a, spVector b, spFloat radius, spColor color, spColor border);
DEMO_API void spDrawCircle(spVector center, spFloat angle, spFloat radius, spColor color, spColor border);
DEMO_API void spDrawSpring(spVector start, spVector end, spFloat linewidth, spFloat springWidth, spColor color, spColor border);
DEMO_API void spDrawRope(spVector start, spVector end, spInt iterations, spFloat size, spColor colorA, spColor colorB, spColor border);
DEMO_API void spClearBuffers();
DEMO_API void spDrawDemo();

DEMO_API extern spRenderContext context;

#endif