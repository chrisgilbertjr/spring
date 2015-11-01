
#include <string.h>
#include "demo\spDraw.h"
#include "demo\spDemo.h"

static const spInt BufferGrowSize = 64;
spRenderContext context;

#define aliasZero { 0.0f, 0.0f }
#define baryZero   { 0.0f, 0.0f, 0.0f }
#define baryBorder { 0.0f, 1.0f, 0.0f }
#define baryCenter { 1.0f, 0.0f, 0.0f }
#define FLUSH_GL_ERRORS() glGetError()
#define BUFFER_OFFSET(index) ((char *)NULL + (index)) 
#define VERTEX(a) (spVertex) {a}

static void 
Transpose(spFloat* transpose)
{
    spFloat tmp[16];

    for (spInt i = 0; i < 16; ++i)
    {
        tmp[i] = transpose[i];
    }

    transpose[0]  = tmp[0];
    transpose[1]  = tmp[1];
    transpose[2]  = tmp[2];
    transpose[3]  = tmp[3];
    transpose[4]  = tmp[4];
    transpose[5]  = tmp[5];
    transpose[6]  = tmp[6];
    transpose[7]  = tmp[7];
    transpose[8]  = tmp[8];
    transpose[9]  = tmp[9];
    transpose[10] = tmp[10];
    transpose[11] = tmp[11];
    transpose[12] = tmp[12];
    transpose[13] = tmp[13];
    transpose[14] = tmp[14];
    transpose[15] = tmp[15];
}

static void
MultMatrix4Vector4(const spFloat m[16], const spFloat v[4], spFloat result[4])
{
    result[0] = v[0]*m[0]  + v[1]*m[1]  + v[2]*m[2]  + v[3]*m[3];
    result[1] = v[0]*m[4]  + v[1]*m[5]  + v[2]*m[6]  + v[3]*m[7];
    result[2] = v[0]*m[8]  + v[1]*m[9]  + v[2]*m[10] + v[3]*m[11];
    result[3] = v[0]*m[12] + v[1]*m[13] + v[2]*m[14] + v[3]*m[15];
}

static void 
MultMatrix4(const spFloat a[16], const spFloat b[16], spFloat result[16])
{
    result[0]  = a[0]*b[0]  +  a[1]*b[4]  + a[2]*b[8]   + a[3]*b[12];
    result[1]  = a[0]*b[1]  +  a[1]*b[5]  + a[2]*b[9]   + a[3]*b[13];
    result[2]  = a[0]*b[2]  +  a[1]*b[6]  + a[2]*b[10]  + a[3]*b[14];
    result[3]  = a[0]*b[3]  +  a[1]*b[7]  + a[2]*b[11]  + a[3]*b[15];

    result[4]  = a[4]*b[0]  +  a[5]*b[4]  + a[6]*b[8]   + a[7]*b[12];
    result[5]  = a[4]*b[1]  +  a[5]*b[5]  + a[6]*b[9]   + a[7]*b[13];
    result[6]  = a[4]*b[2]  +  a[5]*b[6]  + a[6]*b[10]  + a[7]*b[14];
    result[7]  = a[4]*b[3]  +  a[5]*b[7]  + a[6]*b[11]  + a[7]*b[15];

    result[8]  = a[8]*b[0]  +  a[9]*b[4]  + a[10]*b[8]  + a[11]*b[12];
    result[9]  = a[8]*b[1]  +  a[9]*b[5]  + a[10]*b[9]  + a[11]*b[13];
    result[10] = a[8]*b[2]  +  a[9]*b[6]  + a[10]*b[10] + a[11]*b[14];
    result[11] = a[8]*b[3]  +  a[9]*b[7]  + a[10]*b[11] + a[11]*b[15];

    result[12] = a[12]*b[0] +  a[13]*b[4] + a[14]*b[8]  + a[15]*b[12];
    result[13] = a[12]*b[1] +  a[13]*b[5] + a[14]*b[9]  + a[15]*b[13];
    result[14] = a[12]*b[2] +  a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
    result[15] = a[12]*b[3] +  a[13]*b[7] + a[14]*b[11] + a[15]*b[15];
}

static void
InverseMatrix4(const spFloat m[16], spFloat result[16])
{
    /// laplace expansion theorem
    /// http://www.geometrictools.com/Documentation/LaplaceExpansionTheorem.pdf

    /// 2x2 cofactor matrices calculated only once 
    spFloat s0 = m[0] * m[5]  - m[1]  * m[4];
    spFloat s1 = m[0] * m[6]  - m[2]  * m[4];
    spFloat s2 = m[0] * m[7]  - m[3]  * m[4];
    spFloat s3 = m[1] * m[6]  - m[2]  * m[5];
    spFloat s4 = m[1] * m[7]  - m[3]  * m[5];
    spFloat s5 = m[2] * m[7]  - m[3]  * m[6];
    spFloat c0 = m[8] * m[13] - m[9]  * m[12];
    spFloat c1 = m[8] * m[14] - m[10] * m[12];
    spFloat c2 = m[8] * m[15] - m[11] * m[12];
    spFloat c3 = m[9] * m[14] - m[10] * m[13];
    spFloat c4 = m[9] * m[15] - m[11] * m[13];
    spFloat c5 = m[10]* m[15] - m[11] * m[14];

    /// compute the determinant
    spFloat determinant = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;

    /// check if the determinant if position
    if (determinant <= 0.0f) { spMemset(result, 0, 16*sizeof(spFloat)); return; }

    /// get the inv determinant
    float invDeterminant = 1.0f / determinant;

    /// compute the inverse of the matrix
    result[0]  = +(m[5]  * c5 - m[6]  * c4 + m[7]  * c3) * invDeterminant;
    result[1]  = -(m[1]  * c5 - m[2]  * c4 + m[3]  * c3) * invDeterminant;
    result[2]  = +(m[13] * s5 - m[14] * s4 + m[15] * s3) * invDeterminant;
    result[3]  = -(m[9]  * s5 - m[10] * s4 + m[11] * s3) * invDeterminant;

    result[4]  = -(m[4]  * c5 - m[6]  * c2 + m[7]  * c1) * invDeterminant;
    result[5]  = +(m[0]  * c5 - m[2]  * c2 + m[3]  * c1) * invDeterminant;
    result[6]  = -(m[12] * s5 - m[14] * s2 + m[15] * s1) * invDeterminant;
    result[7]  = +(m[8]  * s5 - m[10] * s2 + m[11] * s1) * invDeterminant;

    result[8]  = +(m[4]  * c4 - m[5]  * c2 + m[7]  * c0) * invDeterminant;
    result[9]  = -(m[0]  * c4 - m[1]  * c2 + m[3]  * c0) * invDeterminant;
    result[10] = +(m[12] * s4 - m[13] * s2 + m[15] * s0) * invDeterminant;
    result[11] = -(m[8]  * s4 - m[9]  * s2 + m[11] * s0) * invDeterminant;

    result[12] = -(m[4]  * c3 - m[5]  * c1 + m[6]  * c0) * invDeterminant;
    result[13] = +(m[0]  * c3 - m[1]  * c1 + m[2]  * c0) * invDeterminant;
    result[14] = -(m[12] * s3 - m[13] * s1 + m[14] * s0) * invDeterminant;
    result[15] = +(m[8]  * s3 - m[9]  * s1 + m[10] * s0) * invDeterminant;
}

spVector
spDeproject(spVector position, const spFloat model[16], const spFloat proj[16], spViewport view)
{
    /// project mouse coords from screen space to world space
    spFloat inverse[16] = {0};
    spFloat camera[16] = {0};
    spFloat screen[4] = {(position.x / view.width)  * 2.0f - 1.0f, (position.y / view.height) * 2.0f - 1.0f, 0.0f, 1.0f};
    spFloat world[4] = {0};

    MultMatrix4(proj, model, camera);

    InverseMatrix4(camera, inverse);
    MultMatrix4Vector4(inverse, screen, world);

    /// shouldnt divide by 0
    if (world[3] == 0.0f) { return spVectorZero(); }

    world[0] /= world[3];
    world[1] /= world[3];

    /// return world space coords
    return spVectorConstruct(world[0], world[1]);
}

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
    while (context.triangles >= context.capacity)
    {
        GrowTriangleBuffer();
    }
}

static void
AddTriangle(spVertex* a, spVertex* b, spVertex* c)
{
    UpdateBufferSize();

    spTriangle triangle = { *a, *b, *c };
    context.buffer[context.triangles++] = triangle;
}

/// @}

///* Shader functions
/// @{

static void
CheckGLErrors()
{
    GLenum error = -1;
    while (error = glGetError())
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
    context = (spRenderContext){NULL, 0, 0, 0, 0, 0, 0, BufferGrowSize};
    /// INIT THE CONTEXT HERE
    /// @TODO:
}

static void
SetupShaders()
{
    /// create the shader program
    context.vertexShader  = CreateVertexShader(vertexShader);
    context.pixelShader   = CreatePixelShader(pixelShader);
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
    for(spInt i = 0; i < count; ++i)
    {
        spVector v0 = center;
        spVector v1 = verts[i];
        spVector v2 = verts[(i+1)%count];

        spVertex p0 = (spVertex){{v0.x, v0.y}, {0.0f, -1.0f}, {1.0f, 1.0f, 0.0f}, color, border};
        spVertex p1 = (spVertex){{v1.x, v1.y}, {0.0f,  1.0f}, {0.0f, 1.0f, 0.0f}, color, border}; 
        spVertex p2 = (spVertex){{v2.x, v2.y}, {0.0f,  1.0f}, {0.0f, 1.0f, 1.0f}, color, border};
        AddTriangle(&p0, &p1, &p2);
    }
}

void spDrawSegment(spVector a, spVector b, spFloat radius, spColor color, spColor border)
{
	spVector n = spNormal(spSkew(spSubVecs(b, a)));
	spVector t = spSkew(n);
	
	spFloat r = radius * 0.5f;
	
	spVector nw = spMultVecFlt(n, r);
	spVector tw = spMultVecFlt(t, r);
	spVector v0 = spSubVecs(b, spAddVecs(nw, tw));
	spVector v1 = spAddVecs(b, spSubVecs(nw, tw));
	spVector v2 = spSubVecs(b, nw);
	spVector v3 = spAddVecs(b, nw);
	spVector v4 = spSubVecs(a, nw);
	spVector v5 = spAddVecs(a, nw);
	spVector v6 = spSubVecs(a, spSubVecs(nw, tw));
	spVector v7 = spAddVecs(a, spAddVecs(nw, tw));

    spVertex p0 = {{v0.x, v0.y}, { 1.0f, -1.0f}, baryZero, color, border};
    spVertex p1 = {{v1.x, v1.y}, { 1.0f,  1.0f}, baryZero, color, border};
    spVertex p2 = {{v2.x, v2.y}, { 0.0f, -1.0f}, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);

    p0 = (spVertex){{v3.x, v3.y}, { 0.0f,  1.0f}, baryZero, color, border};
    p1 = (spVertex){{v1.x, v1.y}, { 1.0f,  1.0f}, baryZero, color, border};
    p2 = (spVertex){{v2.x, v2.y}, { 0.0f, -1.0f}, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);

    p0 = (spVertex){{v3.x, v3.y}, { 0.0f,  1.0f}, baryZero, color, border};
    p1 = (spVertex){{v4.x, v4.y}, { 0.0f, -1.0f}, baryZero, color, border}; 
    p2 = (spVertex){{v2.x, v2.y}, { 0.0f, -1.0f}, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);

    p0 = (spVertex){{v3.x, v3.y}, { 0.0f,  1.0f}, baryZero, color, border}; 
    p1 = (spVertex){{v4.x, v4.y}, { 0.0f, -1.0f}, baryZero, color, border}; 
    p2 = (spVertex){{v5.x, v5.y}, { 0.0f,  1.0f}, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);

    p0 = (spVertex){{v6.x, v6.y}, {-1.0f, -1.0f}, baryZero, color, border}; 
    p1 = (spVertex){{v4.x, v4.y}, { 0.0f, -1.0f}, baryZero, color, border}; 
    p2 = (spVertex){{v5.x, v5.y}, { 0.0f,  1.0f}, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);

    p0 = (spVertex){{v6.x, v6.y}, {-1.0f, -1.0f}, baryZero, color, border}; 
    p1 = (spVertex){{v7.x, v7.y}, {-1.0f,  1.0f}, baryZero, color, border}; 
    p2 = (spVertex){{v5.x, v5.y}, { 0.0f,  1.0f}, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);
}

void
spDrawLine(spVector start, spVector end, spFloat size, spColor color, spColor border)
{
    spVector a = start;
    spVector b = end;
    spVector n = spNormal(spSkew(spSubVecs(b, a)));

    spFloat r = size * 0.5f;

    spVector nw = spMultVecFlt(n, r);
    spVector v2 = spSubVecs(b, nw);
    spVector v3 = spAddVecs(b, nw);
    spVector v4 = spSubVecs(a, nw);
    spVector v5 = spAddVecs(a, nw);

    spVertex p0 = {{ v3.x, v3.y }, { 0.0f, 1.0f }, baryZero, color, border};
    spVertex p1 = {{ v4.x, v4.y }, { 0.0f,-1.0f }, baryZero, color, border};
    spVertex p2 = {{ v2.x, v2.y }, { 0.0f,-1.0f }, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);

    p0 = (spVertex){{ v3.x, v3.y }, { 0.0f, 1.0f }, baryZero, color, border};
    p1 = (spVertex){{ v4.x, v4.y }, { 0.0f,-1.0f }, baryZero, color, border};
    p2 = (spVertex){{ v5.x, v5.y }, { 0.0f, 1.0f }, baryZero, color, border};
    AddTriangle(&p0, &p1, &p2);
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

static spVector spring[12] = 
{
    { 0.0f, 0.00f},
    { 0.0f, 0.10f},
    { 0.5f, 0.15f},
    {-1.0f, 0.25f},
    { 1.0f, 0.35f},
    {-1.0f, 0.45f},
    { 1.0f, 0.55f},
    {-1.0f, 0.65f},
    { 1.0f, 0.75f},
    {-0.5f, 0.85f},
    { 0.0f, 0.90f},
    { 0.0f, 1.00f}
};

void 
spDrawSpring(spVector start, spVector end, spFloat linewidth, spFloat springWidth, spColor color, spColor border)
{
    spVector tangent = spSkew(spNormal(spSubVecs(end, start)));
    spVector pointA = spAddVecs(spMultVecFlt(tangent, spring[0].x * springWidth), spLerpVec(start, end, spring[0].y));
    for (spInt i = 1; i < 12; ++i)
    {
        spVector pointB = spAddVecs(spMultVecFlt(tangent, spring[i].x * springWidth), spLerpVec(start, end, spring[i].y));
        spDrawSegment(pointA, pointB, linewidth, color, color);
        pointA = pointB;
    }
}

void 
spDrawRope(spVector start, spVector end, spInt segments, spFloat size, spColor colorA, spColor colorB, spColor border)
{
    spVector tangent = spNormal(spSkew(spSubVecs(start, end)));
    spFloat segment = 1.0f / (spFloat)segments;

    spColor color0 = colorA;
    spColor color1 = colorB;

    for (spInt i = 0; i < segments; ++i)
    {
        spFloat iter0 = (spFloat)i;
        spFloat iter1 = (spFloat)i+1;

        spVector pointA = spLerpVec(start, end, iter0*segment);
        spVector pointB = spLerpVec(start, end, iter1*segment);

        spFloat r = size * 0.5f;

        spVector offset = spMultVecFlt(tangent, r);
        spVector v2 = spSubVecs(pointB, offset);
        spVector v3 = spAddVecs(pointB, offset);
        spVector v4 = spSubVecs(pointA, offset);
        spVector v5 = spAddVecs(pointA, offset);

        spVertex p0 = { { v3.x, v3.y }, { 0.0f, 1.0f }, baryZero, color0, border };
        spVertex p1 = { { v4.x, v4.y }, { 0.0f, -1.0f }, baryZero, color0, border };
        spVertex p2 = { { v2.x, v2.y }, { 0.0f, -1.0f }, baryZero, color0, border };
        AddTriangle(&p0, &p1, &p2);

        p0 = (spVertex){{ v3.x, v3.y }, { 0.0f, 1.0f }, baryZero, color1, border};
        p1 = (spVertex){{ v4.x, v4.y }, { 0.0f,-1.0f }, baryZero, color1, border};
        p2 = (spVertex){{ v5.x, v5.y }, { 0.0f, 1.0f }, baryZero, color1, border};
        AddTriangle(&p0, &p1, &p2);;

        /// swap the colors
        spColor tmp = color0;
        color0 = color1;
        color1 = tmp;
    }
}

void spClearBuffers()
{
    glClearColor(Demo->background.r, Demo->background.g, Demo->background.b, Demo->background.a);
    context.triangles = 0;
    glClear(GL_COLOR_BUFFER_BIT);
}

void spDrawDemo()
{
    /// map the triangles into graphics memory
    MapBuffers();

    glUseProgram(context.shaderProgram);

    spFloat transform[16];
    MultMatrix4(Demo->ortho, Demo->view, transform);

    glUniformMatrix4fv(glGetUniformLocation(context.shaderProgram, "v_transform"), 1, GL_TRUE, transform);

    /// bind the vertex array and draw the scene
    glBindVertexArray(context.vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, context.vertexBuffer);
    glDrawArrays(GL_TRIANGLES, 0, context.triangles*3);
    glBindVertexArray(0);
}

/// @}