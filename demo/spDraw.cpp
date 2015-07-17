
#include "spDraw.h"
#include "spDemo.h"

static const spInt BufferGrowSize = 16;
spRenderContext context;

#define aliasZero { 0.0f, 0.0f }
#define baryZero { 0.0f, 0.0f, 0.0f }
#define FLUSH_GL_ERRORS() glGetError()

static void
InitOpenGL()
{
    glewExperimental = GL_TRUE;
    GLenum error = glewInit();
    if (error != GLEW_OK)
    {
        spWarning(spFalse, "Error: %s\n", glewGetErrorString(error));
        spAssert(spFalse, "Error: cannot init GLEW.\n");
    }
    FLUSH_GL_ERRORS();

    glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
}

static void
InitRenderContext()
{
    context = {NULL, NULL, NULL, NULL, NULL, NULL, 0, BufferGrowSize};
    /// INIT THE CONTEXT HERE
}

static void
ReallocBuffers()
{
    void* mem = spRealloc(context.buffer, sizeof(spTriangle) * context.capacity);
    spAssert(mem != NULL, "triangle buffer reallocation failed!\n");
    context.buffer = (spTriangle*) mem;

    glBindBuffer(GL_ARRAY_BUFFER, context.vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(spTriangle) * context.capacity, 0, GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

static void
GrowTriangleBuffer()
{
    context.capacity += BufferGrowSize;
    ReallocBuffers();
}

static void
ShrinkTriangleBuffer()
{
    context.capacity = context.capacity / 2;
    ReallocBuffers();
}

static void
UpdateBufferSize()
{
    while (context.capacity <= BufferGrowSize)
    {
        if (context.triangles >= context.capacity)
    	{
    	    GrowTriangleBuffer();
    	}
    	else if (context.triangles >= context.capacity/2)
    	{
    	    ShrinkTriangleBuffer();
    	}
    	else 
    	{ 
    	    break; 
    	}
    }
}

static void
addTriangle(spVertex* a, spVertex* b, spVertex* c)
{
    UpdateBufferSize();

    context.buffer[context.triangles] = {*a, *b, *c};
}

void spDrawInit()
{
    InitOpenGL();
    InitRenderContext();
}

void spDrawPolygon()
{
}

void spDrawSegment()
{
}

void spDrawCircle(spVector center, spFloat angle, spFloat radius, spColor fill, spColor outline)
{
    spVertex a = {{center.x - radius, center.y - radius}, {-1.0f,-1.0f}, baryZero, fill, outline};
    spVertex b = {{center.x + radius, center.y - radius}, {+1.0f,-1.0f}, baryZero, fill, outline};
    spVertex c = {{center.x + radius, center.y + radius}, {+1.0f,+1.0f}, baryZero, fill, outline};
    spVertex d = {{center.x - radius, center.y + radius}, {-1.0f,+1.0f}, baryZero, fill, outline};

    addTriangle(&a, &b, &c);
    addTriangle(&c, &d, &a);
}