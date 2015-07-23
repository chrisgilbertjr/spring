
#ifndef SP_DEBUG_DRAW_GL2_H
#define SP_DEBUG_DRAW_GL2_H

#include <GL\glew.h>
#include "spShader.h"
#include "spWorld.h"

struct spVec { GLfloat x, y; };
struct spBary { GLfloat x, y, z; };
struct spColor { GLfloat r, g, b, a; };
struct spVertex { spVec pos, aliasing; spBary barycentric; spColor fill, outline; };
struct spTriangle { spVertex a, b, c; };
struct spViewport { spFloat width, height; };
struct spFrustum { spFloat left, right, top, bottom, near, far; };

/// some useful colors
#define RGB(r, g, b)     {(GLfloat)r, (GLfloat)g, (GLfloat)b, (GLfloat)1 }
#define RGBA(r, g, b, a) {(GLfloat)r, (GLfloat)g, (GLfloat)b, (GLfloat)a }
#define COLA(c, a)       {(GLfloat)c, (GLfloat)c, (GLfloat)c, (GLfloat)a }

#define RED()    RGBA(0.5f, 0.0f, 0.0f, 1.0) 
#define BLUE()   RGBA(0.0f, 0.0f, 0.5f, 1.0) 
#define GREEN()  RGBA(0.0f, 0.5f, 0.0f, 1.0) 
#define YELLOW() RGBA(0.5f, 0.5f, 0.0f, 1.0) 
#define PURPLE() RGBA(0.5f, 0.0f, 0.5f, 1.0) 
#define BLACK()  COLA(0.0f, 1.0) 
#define WHITE()  COLA(1.0f, 1.0) 

struct spRenderContext
{
    spTriangle* buffer;
    GLuint vertexArray, vertexBuffer;
    GLint vertexShader, pixelShader, shaderProgram;
    GLint triangles, capacity;
};

spVector spDeproject(spVector position, const spFloat model[16], const spFloat proj[16], spViewport view);
    
void spDrawInit();
void spDrawLine(spVector start, spVector end, spFloat size, spColor color, spColor border);
void spDrawPolygon(spVector position, spFloat angle, spVector* verts, spInt count, spVector center, spColor color, spColor border);
void spDrawSegment(spVector a, spVector b, spFloat radius, spColor color, spColor border);
void spDrawCircle(spVector center, spFloat angle, spFloat radius, spColor color, spColor border);
void spDrawSpring(spVector start, spVector end, spFloat linewidth, spFloat springWidth, spColor color, spColor border);
void spDrawRope(spVector start, spVector end, spInt iterations, spFloat size, spColor colorA, spColor colorB, spColor border);

void spClearBuffers();
void spDrawDemo();

extern spRenderContext context;

#endif