
#include <GL\glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include "spCore.h"
#include <cstring>
#include "spMath.h"
#include "spShader.h"

#define STRINGIFY(string) #string
#define BUFFER_OFFSET(i) ((char *)NULL + (i)) 
#define FLUSH_ERRORS() glGetError()

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

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
    glGetProgramInfoLog(object, length, (GLsizei*)0, infoLog);
    spWarning(spFalse, "Info Log: %s\n", infoLog);
    spFree(&infoLog);
}

static spBool
CompileSuccessful(GLint shader)
{
    /// get the compilse status
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    
    /// if successful, return true
    if (success)
    {
        return spTrue;
    }

    /// compile failed, get the info log and pring it
    GLint length;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    PrintInfoLog(shader, length);

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

    return spFalse;
}

static GLint
CompileShader(GLenum type, const char* source)
{
    GLint shader = glCreateShader(type);
    GLint length = (GLint)strlen(source);

    glShaderSource(shader, 1, &source, &length);
    glCompileShader(shader);

    spAssert(CompileSuccessful(shader), "shader compilation failed\n");
    CheckGLErrors();
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
    CheckGLErrors();
    return program;
}

static GLint
CreateShaderProgramSrc(const char* vertex, const char* pixel)
{
    return CreateShaderProgram(CreateVertexShader(vertex), CreatePixelShader(pixel));
}

struct vector { GLfloat x; GLfloat y; };
struct bary { GLfloat x; GLfloat y; GLfloat z; };
struct vertex { vector pos; vector aliasing; bary bary2; };
struct triangle { vertex a, b, c; };
#define aliasZero { 0.0f, 0.0f }
#define baryZero { 0.0f, 0.0f, 0.0f }

static GLint count = 0;
static const GLint triangles = 24;
static triangle data[triangles];

static void 
addTriangle(vertex a, vertex b, vertex c)
{
    data[count++] = {a, b, c};
}

static void 
spDrawCircle(spVector center, spFloat angle, spFloat radius)
{
    vertex a = {{center.x - radius, center.y - radius}, {-1.0f,-1.0f}, baryZero};
    vertex b = {{center.x + radius, center.y - radius}, {+1.0f,-1.0f}, baryZero};
    vertex c = {{center.x + radius, center.y + radius}, {+1.0f,+1.0f}, baryZero};
    vertex d = {{center.x - radius, center.y + radius}, {-1.0f,+1.0f}, baryZero};

    addTriangle(a, b, c);
    addTriangle(c, d, a);
}

static void
spDrawCapsule(spVector pos, spVector start, spVector end, spFloat radius)
{
    vertex a = {{start.x, start.y - radius}, aliasZero, {1.0f, 0.0f, 0.0f}};
    vertex b = {{end.x  , end.y   - radius}, aliasZero, {1.0f, 1.0f, 0.0f}};
    vertex c = {{end.x  , end.y   + radius}, aliasZero, {0.0f, 1.0f, 1.0f}};
    addTriangle(a, b, c);

    a = {{end.x  , end.y   + radius}, aliasZero, {1.0f, 0.0f, 0.0f}};
    b = {{start.x, start.y + radius}, aliasZero, {1.0f, 1.0f, 0.0f}};
    c = {{start.x, start.y - radius}, aliasZero, {0.0f, 1.0f, 1.0f}};
    addTriangle(a, b, c);

    a        = {{start.x - radius, start.y - radius}, {-1.0f,-1.0f}, {1.0f, 1.0f, 0.0f}};
    b        = {{start.x         , start.y - radius}, { 0.0f,-1.0f}, {1.0f, 1.0f, 0.0f}};
    c        = {{start.x         , start.y + radius}, { 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    vertex d = {{start.x - radius, start.y + radius}, {-1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}};
    addTriangle(a, b, c);
    addTriangle(c, d, a);

    a = {{end.x + radius, end.y - radius}, {-1.0f,-1.0f}, {1.0f, 1.0f, 0.0f}};
    b = {{end.x         , end.y - radius}, { 0.0f,-1.0f}, {1.0f, 1.0f, 0.0f}};
    c = {{end.x         , end.y + radius}, { 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    d = {{end.x + radius, end.y + radius}, {-1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}};
    addTriangle(a, b, c);
    addTriangle(c, d, a);
}

int main(void)
{
    GLFWwindow* window;
    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    window = glfwCreateWindow(800, 800, "Simple example", NULL, NULL);

    if (!window)
    {
        spAssert(spFalse, "\nWindow creationg failed.\n");
        glfwTerminate();
        exit(1);
    }

    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
        spAssert(spFalse, "Error: cannot init GLEW.\n");
    }
    FLUSH_ERRORS();

    glfwSetKeyCallback(window, key_callback);

    glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    GLuint vao;
    GLuint vbo;
    GLint program = CreateShaderProgramSrc(vertexShape, pixelShape);

    glUseProgram(program);

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(triangle) * triangles, 0, GL_STREAM_DRAW);

    GLint index = glGetAttribLocation(program, "v_position");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), 0);

    index = glGetAttribLocation(program, "v_aliasing");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(sizeof(vector)));

    index = glGetAttribLocation(program, "v_barycentric");
	glEnableVertexAttribArray(index);
	glVertexAttribPointer(index, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(sizeof(vector)*2));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT);

        spDrawCircle({0.0f, 0.0f}, 0.0f, 0.1f);
        spDrawCapsule({0.5f, 0.1f}, {-0.1f, 0.0f}, {0.5f, 0.5f}, 0.2);

        GLint size = sizeof(triangle) * count;
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        void* mem = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
        memcpy(mem, data, size);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glDrawArrays(GL_TRIANGLES, 0, size);
        glBindVertexArray(0);

        glfwSwapBuffers(window);
        glfwPollEvents();
        count = 0;
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}