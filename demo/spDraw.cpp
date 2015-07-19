
#include <cstring>
#include "spDraw.h"
#include "spDemo.h"

static const spInt BufferGrowSize = 16;
spRenderContext context;

#define aliasZero { 0.0f, 0.0f }
#define baryZero { 0.0f, 0.0f, 0.0f }
#define FLUSH_GL_ERRORS() glGetError()
#define BUFFER_OFFSET(index) ((char *)NULL + (index)) 

///* Render context buffer functions
/// @{

static void
SetupBuffers()
{
    context.buffer = (spTriangle*) spMalloc(sizeof(spTriangle) * context.capacity);

    glGenVertexArrays(1, &context.vertexArray);
    glBindVertexArray(context.vertexArray);

    glGenBuffers(1, &context.vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, context.vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(spTriangle) * context.capacity, 0, GL_STREAM_DRAW);

    GLint offset = 0;
    GLint index = glGetAttribLocation(context.shaderProgram, "v_position");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 2, GL_FLOAT, GL_FALSE, sizeof(spVertex), BUFFER_OFFSET(offset));

    offset += sizeof(spVec);
    index = glGetAttribLocation(context.shaderProgram, "v_aliasing");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 2, GL_FLOAT, GL_FALSE, sizeof(spVertex), BUFFER_OFFSET(offset));

    offset += sizeof(spVec);
    index = glGetAttribLocation(context.shaderProgram, "v_barycentric");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 3, GL_FLOAT, GL_FALSE, sizeof(spVertex), BUFFER_OFFSET(offset));

    offset += sizeof(spBary);
    index = glGetAttribLocation(context.shaderProgram, "v_color");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 4, GL_FLOAT, GL_FALSE, sizeof(spVertex), BUFFER_OFFSET(offset));

    offset += sizeof(spColor);
    index = glGetAttribLocation(context.shaderProgram, "v_border");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 4, GL_FLOAT, GL_FALSE, sizeof(spVertex), BUFFER_OFFSET(offset));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

static void
MapBuffers()
{
    glBindBuffer(GL_ARRAY_BUFFER, context.vertexBuffer);
    void* mem = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(mem, context.buffer, sizeof(spTriangle) * context.triangles);
    glUnmapBuffer(GL_ARRAY_BUFFER);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
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
    	else 
    	{ 
    	    break; 
    	}
    }
}

static void
AddTriangle(spVertex* a, spVertex* b, spVertex* c)
{
    UpdateBufferSize();

    context.buffer[context.triangles++] = {*a, *b, *c};
}

/// @}

///* Shader functions
/// @{

static void
CheckGLErrors()
{
    while (GLenum error = glGetError())
    {
        if (error)
        {
            spWarning(error, "OpenGL Error: %s:%d - %s\n", 
                __FILE__, 
                __LINE__, 
                (char*)glewGetErrorString(error));
        }
    }
}

static void
PrintInfoLog(GLint object, GLint length)
{
    char* infoLog = (char*)spMalloc(length);
    glGetProgramInfoLog(object, length, (GLsizei*)NULL, infoLog);
    spWarning(spFalse, "Info Log: %s\n", infoLog);
    spFree(&infoLog);
}

static spBool
CompileSuccessful(GLint shader)
{
    /// get the compile status
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    
    /// if successful, return true
    if (success) { return spTrue; }

    /// compile failed, get the info log and pring it
    GLint length;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    PrintInfoLog(shader, length);
    CheckGLErrors();

    return spFalse;
}

static spBool
LinkSuccessful(GLint program)
{
    /// get the link status
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    
    /// if successful, return true
    if (success)
    {
        return spTrue;
    }

    /// link failed, get the info log and print it
    GLint length;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    PrintInfoLog(program, length);
    CheckGLErrors();

    return spFalse;
}

static GLint
CompileShader(GLenum type, const char* source)
{
    /// create the shader
    GLint shader = glCreateShader(type);
    GLint length = (GLint)strlen(source);

    /// feed in the shader source and compule the shader
    glShaderSource(shader, 1, &source, &length);
    glCompileShader(shader);

    /// check if the compilewas successful
    spAssert(CompileSuccessful(shader), "shader compilation failed\n");
    return shader;
}

static GLint
CreateVertexShader(const char* source)
{
    return CompileShader(GL_VERTEX_SHADER, source);
}

static GLint
CreatePixelShader(const char* source)
{
    return CompileShader(GL_FRAGMENT_SHADER, source);
}

static GLint
CreateShaderProgram(GLint vertex, GLint pixel)
{
    GLint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, pixel);
    glLinkProgram(program);

    spAssert(LinkSuccessful(program), "linking program failed\n");
    return program;
}

/// @}

///* Initialization functions
/// @{

static void
SetupOpenGL()
{
    /// init glew for core profile
    glewExperimental = GL_TRUE;
    GLenum error = glewInit();

    /// make sure glew init successfully
    if (error != GLEW_OK)
    {
        spWarning(spFalse, "Error: %s\n", glewGetErrorString(error));
        spAssert(spFalse, "Error: cannot init GLEW.\n");
    }
    FLUSH_GL_ERRORS();

    /// setup GL for alpha blending
    glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
}

static void
SetupRenderContext()
{
    context = {NULL, NULL, NULL, NULL, NULL, NULL, 0, BufferGrowSize};
    /// INIT THE CONTEXT HERE
    /// @TODO:
}

static void
SetupShaders()
{
    /// create the shader program
    context.vertexShader  = CreateVertexShader(vertexShape);
    context.pixelShader   = CreatePixelShader(pixelShape);
    context.shaderProgram = CreateShaderProgram(context.vertexShader, context.pixelShader);

    SetupBuffers();
}

/// @}

/// @defgroup spDraw spDraw
/// @{

void spDrawInit()
{
    SetupOpenGL();
    SetupRenderContext();
    SetupShaders();
}

void spDrawPolygon(spVector position, spFloat angle, spVector* verts, spInt count, spVector center, spColor color, spColor border)
{
    spTransform xf = spTransformConstruct(position, spRotationConstruct(angle));
    spVector v0 = spMult(xf, center);
    for(spInt i = 0; i < count; ++i)
    {
        spVector v1 = spMult(xf, verts[i]);
        spVector v2 = spMult(xf, verts[(i+1)%count]);

        spVertex a = {{v0.x, v0.y}, aliasZero, {1.0f, 1.0f, 1.0f}, color, border};
    	spVertex b = {{v1.x, v1.y}, aliasZero, {0.0f, 1.0f, 1.0f}, color, border};
    	spVertex c = {{v2.x, v2.y}, aliasZero, {0.0f, 1.0f, 1.0f}, color, border};
        AddTriangle(&a, &b, &c);
    }
}

void spDrawSegment()
{
}

void spDrawCircle(spVector center, spFloat angle, spFloat radius, spColor color, spColor border)
{
    spVertex a = {{center.x - radius, center.y - radius}, {-1.0f,-1.0f}, baryZero, color, border};
    spVertex b = {{center.x + radius, center.y - radius}, {+1.0f,-1.0f}, baryZero, color, border};
    spVertex c = {{center.x + radius, center.y + radius}, {+1.0f,+1.0f}, baryZero, color, border};
    spVertex d = {{center.x - radius, center.y + radius}, {-1.0f,+1.0f}, baryZero, color, border};

    AddTriangle(&a, &b, &c);
    AddTriangle(&c, &d, &a);
}

void spClearBuffers()
{
    context.triangles = 0;
    glClear(GL_COLOR_BUFFER_BIT);
}

void spDrawDemo()
{
    /// map the triangles into graphics memory
    MapBuffers();

    glUseProgram(context.shaderProgram);

    /// bind the vertex array and draw the scene
    glBindVertexArray(context.vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, context.vertexBuffer);
    glDrawArrays(GL_TRIANGLES, 0, sizeof(spTriangle) * context.triangles);
    glBindVertexArray(0);
}

/// @}