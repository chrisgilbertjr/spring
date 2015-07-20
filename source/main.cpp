
#include <GL\glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include "spCore.h"
#include <cstring>
#include "spMath.h"
#include "spShader.h"
#include "spDraw.h"
#include "spDemo.h"

#define STRINGIFY(string) #string
#define BUFFER_OFFSET(i) ((char *)NULL + (i)) 
#define FLUSH_ERRORS() glGetError()

//static void 
//spDrawCircle(spVector center, spFloat angle, spFloat radius)
//{
//    vertex a = {{center.x - radius, center.y - radius}, {-1.0f,-1.0f}, baryZero};
//    vertex b = {{center.x + radius, center.y - radius}, {+1.0f,-1.0f}, baryZero};
//    vertex c = {{center.x + radius, center.y + radius}, {+1.0f,+1.0f}, baryZero};
//    vertex d = {{center.x - radius, center.y + radius}, {-1.0f,+1.0f}, baryZero};

//    addTriangle(a, b, c);
//    addTriangle(c, d, a);
//}

//static void
//spDrawCapsule(spVector pos, spVector start, spVector end, spFloat radius)
//{
//    vertex a = {{start.x, start.y - radius}, aliasZero, {1.0f, 0.0f, 0.0f}};
//    vertex b = {{  end.x,   end.y - radius}, aliasZero, {1.0f, 1.0f, 0.0f}};
//    vertex c = {{  end.x,   end.y + radius}, aliasZero, {0.0f, 1.0f, 1.0f}};
//    vertex d = {{start.x, start.y + radius}, aliasZero, {0.0f, 1.0f, 1.0f}};
//    vertex e = {{start.x - radius, start.y - radius}, {-1.0f,-1.0f}, {1.0f, 1.0f, 0.0f}};
//    vertex f = {{start.x - radius, start.y + radius}, {-1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}};
//    vertex g = {{end.x + radius, end.y - radius}, {-1.0f,-1.0f}, {1.0f, 1.0f, 0.0f}};
//    vertex h = {{end.x + radius, end.y + radius}, {-1.0f, 1.0f}, {1.0f, 1.0f, 0.0f}};

//    /// draw quad body
//    addTriangle(a, b, c);
//    addTriangle(c, d, a);

//    a.aliasing = {0.0f,-1.0f};
//    b.aliasing = {0.0f,-1.0f};
//    c.aliasing = {0.0f, 1.0f};
//    d.aliasing = {0.0f, 1.0f};

//    /// draw endcaps
//    addTriangle(e, a, d);
//    addTriangle(d, f, e);
//    addTriangle(g, b, c);
//    addTriangle(c, h, g);
//}

//static void
//spDrawPolygon(spTransform xf, spVector* verts, spInt size, spVector center)
//{
//    spVector v0 = spMult(xf, center);
//    for(spInt i = 0; i < size; ++i)
//    {
//        spVector v1 = spMult(xf, verts[i]);
//        spVector v2 = spMult(xf, verts[(i+1)%size]);

//        vertex a = {{v0.x, v0.y}, aliasZero, {1.0f, 1.0f, 1.0f}};
//    	vertex b = {{v1.x, v1.y}, aliasZero, {0.0f, 1.0f, 1.0f}};
//    	vertex c = {{v2.x, v2.y}, aliasZero, {0.0f, 1.0f, 1.0f}};
//        addTriangle(a, b, c);
//    }
//}

//static void
//spDrawLine(spVector start, spVector end, spFloat size)
//{
//    spFloat h = size * 0.5f;
//    spVector normal = spNormal(spSkew(spSub(end, start)));
//    spVector offset = spMult(normal, h);
//    spVector v0 = spSub(start, offset);
//    spVector v1 = spSub(  end, offset);
//    spVector v2 = spAdd(  end, offset);
//    spVector v3 = spAdd(start, offset);


//    vertex a = {{v0.x, v0.y}, aliasZero, {1.0f, 1.0f, 0.0f}};
//    vertex b = {{v1.x, v1.y}, aliasZero, {0.0f, 1.0f, 0.0f}};
//    vertex c = {{v2.x, v2.y}, aliasZero, {0.0f, 1.0f, 1.0f}};
//    vertex d = {{v3.x, v3.y}, aliasZero, {0.0f, 1.0f, 0.0f}};

//    addTriangle(a, b, c);

//    c.bary2 = {1.0f, 1.0f, 0.0f};
//    a.bary2 = {0.0f, 1.0f, 1.0f};

//    addTriangle(c, d, a);
//}

//#include <spring.h> 
//#include "spApplication.h"

//#define PRESSED(key) glfwGetKey(app->window, key) == GLFW_PRESS
//#define RELEASED(key) glfwGetKey(app->window, key) == GLFW_RELEASE

//void init_test(spApplication* app)
//{
//    spBody* body;
//    spShape* box;
//    spFilter filter = spFilterCollideAll;

//    spVector vertices[4];
//    spVector size = spVectorConstruct(250.0f, 25.0f);
//    spFloat mass = 25.0f;
//    vertices[0] = spVectorConstruct(-size.x,-size.y); 
//    vertices[1] = spVectorConstruct( size.x,-size.y);
//    vertices[2] = spVectorConstruct( size.x, size.y);
//    vertices[3] = spVectorConstruct(-size.x, size.y);

//    body = spBodyNewStatic();
//    box = spPolygonNew(vertices, 4, mass);
//    box->material.restitution = 0.2f;
//    box->material.friction = 0.8f;
//    spShapeSetFilter(box, filter);
//    spBodySetTransform(body, spVectorConstruct(0.0f, -100.0f), 0.0f);
//    spBodyAddShape(body, box);
//    spWorldAddBody(&app->world, body);

//    spInt num = 6;
//    spFloat w = 20.0f;
//    spFloat h = 10.0f;
//    for (spInt i = 0; i < num; ++i)
//    {
//        spFloat o = w * 2.2f;
//        spFloat p = (spFloat)i;

//        spFloat offsetx = (-num* o * 0.5f) + p*o * 2;

//        mass = 250.0f;
//        size = spVectorConstruct(p+12.0f, p+15.0f);
//        vertices[0] = spVectorConstruct(-size.x,-size.y); 
//        vertices[1] = spVectorConstruct( size.x,-size.y);
//        vertices[2] = spVectorConstruct( size.x, size.y);
//        vertices[3] = spVectorConstruct(-size.x, size.y);

//        body = spBodyNewDynamic();
//        box = spPolygonNew(vertices, 4, mass);
//        box->material.restitution = 0.2f;
//        box->material.friction = 0.8f;
//        spBodySetTransform(body, spVectorConstruct(offsetx, 80.0f), w);
//        spBodyAddShape(body, box);
//        spWorldAddBody(&app->world, body);
//    }

//    body = spBodyNewStatic();
//    box = spSegmentNew(spVectorConstruct(-10.0f, 0.0f), spVectorConstruct(10.0f, 0.0f), 5.0f, 10.0f);
//    box->body = body;
//    spBodySetTransform(body, spVectorConstruct(150.0f, 50.0f), 0.0f);
//    spBodyAddShape(body, box);
//    spWorldAddBody(&app->world, body);

//    body = spBodyNewDynamic();
//    box = spCircleNew(spVectorConstruct(0.0f, 0.0f), 40.0f, 100.0f);
//    box->material.restitution = 0.4f;
//    box->material.friction = 0.5f;
//    spBodySetTransform(body, spVectorConstruct(50.0f, 200.0f), 0.0f);
//    spBodyAddShape(body, box);
//    spWorldAddBody(&app->world, body);

//    body = spBodyNewDynamic();
//    box = spCircleNew(spVectorConstruct(0.0f, 0.0f), 12.0f, 100.0f);
//    box->material.restitution = 0.4f;
//    box->material.friction = 0.5f;
//    spBodySetTransform(body, spVectorConstruct(50.0f, 20.0f), 0.0f);
//    spBodyAddShape(body, box);
//    spWorldAddBody(&app->world, body);

//    body = spBodyNewStatic();
//    box = spSegmentNew(spVectorConstruct(-20.0f, 0.0f), spVectorConstruct(20.0f, 0.0f), 10.0f, 10.0f);
//    box->material.restitution = 0.2f;
//    box->material.friction = 0.8f;
//    spBodySetTransform(body, spVectorConstruct(0.0f, 50.0f), 0.0f);
//    spBodyAddShape(body, box);
//    spWorldAddBody(&app->world, body);
//}

//spApplication* test()
//{
//    return spApplicationNew(
//        "test app",
//        spViewport(800, 800), spFrustumUniform(250),
//        spVectorConstruct(0.f, -98.f), 5, 1.f / 60.f,
//        init_test, default_loop, default_main_loop, NULL);
//}

int main()
{
    spRunDemo(0);
    return 0;
}

//int main2(void)
//{
//    GLFWwindow* window;
//    glfwSetErrorCallback(error_callback);

//    if (!glfwInit())
//    {
//        exit(EXIT_FAILURE);
//    }

//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//    
//    window = glfwCreateWindow(800, 800, "Simple example", NULL, NULL);

//    if (!window)
//    {
//        spAssert(spFalse, "\nWindow creationg failed.\n");
//        glfwTerminate();
//        exit(1);
//    }

//    glfwMakeContextCurrent(window);

//    glewExperimental = GL_TRUE;
//    GLenum err = glewInit();
//    if (err != GLEW_OK)
//    {
//        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
//        spAssert(spFalse, "Error: cannot init GLEW.\n");
//    }
//    FLUSH_ERRORS();

//    glfwSetKeyCallback(window, key_callback);

//    glEnable(GL_BLEND);
//	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

//    GLuint vao;
//    GLuint vbo;
//    GLint program = CreateShaderProgramSrc(vertexShape, pixelShape);

//    glUseProgram(program);

//    glGenVertexArrays(1, &vao);
//    glBindVertexArray(vao);

//    glGenBuffers(1, &vbo);
//    glBindBuffer(GL_ARRAY_BUFFER, vbo);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(triangle) * triangles, 0, GL_STREAM_DRAW);

//    GLint index = glGetAttribLocation(program, "v_position");
//	glEnableVertexAttribArray(index);
//	glVertexAttribPointer(index, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), 0);

//    index = glGetAttribLocation(program, "v_aliasing");
//	glEnableVertexAttribArray(index);
//	glVertexAttribPointer(index, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(sizeof(vector)));

//    index = glGetAttribLocation(program, "v_barycentric");
//	glEnableVertexAttribArray(index);
//	glVertexAttribPointer(index, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), BUFFER_OFFSET(sizeof(vector)*2));

//    glBindBuffer(GL_ARRAY_BUFFER, 0);
//    glBindVertexArray(0);

//    spFloat s = 0.1f;
//    spVector poly[4] = {{-s,-s}, {s,-s}, {s,s}, {-s,s}};

//    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//    while (!glfwWindowShouldClose(window))
//    {
//        count = 0;
//        glClear(GL_COLOR_BUFFER_BIT);


//        GLint size = sizeof(triangle) * count;
//        glBindVertexArray(vao);
//        glBindBuffer(GL_ARRAY_BUFFER, vbo);
//        void* mem = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
//        memcpy(mem, data, size);
//        glUnmapBuffer(GL_ARRAY_BUFFER);
//        glBindBuffer(GL_ARRAY_BUFFER, 0);

//        glDrawArrays(GL_TRIANGLES, 0, size);
//        glBindVertexArray(0);

//        glfwSwapBuffers(window);
//        glfwPollEvents();
//    }

//    glfwDestroyWindow(window);
//    glfwTerminate();
//    exit(EXIT_SUCCESS);
//}