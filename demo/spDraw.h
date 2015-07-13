
#ifndef SP_DEBUG_DRAW_GL2_H
#define SP_DEBUG_DRAW_GL2_H

#include "spCore.h"
#include "spMath.h"
#include <GLFW\glfw3.h>

struct spTriangle
{
    spVector a, b, c;
};

struct spRenderContext
{
    GLuint vertexArray;
    GLuint vertexBuffer;
    GLint shaderProgram;
    GLint vertexShader;
    GLint pixelShader;

    spTriangle* buffer;
    spInt count;
};
    
void spDrawInit();
void spDrawPolygon();
void spDrawSegment();
void spDrawCircle();
void spDrawPoint();
void spDrawCircle();

#endif