
#ifndef SP_DEBUG_DRAW_GL2_H
#define SP_DEBUG_DRAW_GL2_H

#include <GL\glew.h>
#include "spCore.h"
#include "spMath.h"

struct spVec { GLfloat x, y; };
struct spBary { GLfloat x, y, z; };
struct spColor { GLfloat r, g, b, a; };
struct spVertex { spVec pos, aliasing; spBary barycentric; spColor fill, outline; GLfloat PADDING; };
struct spTriangle { spVertex a, b, c; };

struct spRenderContext
{
    GLuint vertexArray;
    GLuint vertexBuffer;
    GLint shaderProgram;
    GLint vertexShader;
    GLint pixelShader;

    spTriangle* buffer;
    spInt triangles, capacity;
};
    
void spDrawInit();
void spDrawPolygon();
void spDrawSegment();
void spDrawCircle(spVector center, spFloat angle, spFloat radius, spColor fill, spColor outline);

extern spRenderContext context;

#endif