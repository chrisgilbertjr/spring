
#include "spDemoFactory.h"

spConstraint* constraints[100] = {NULL};
spDemoShape shapes[100] = {NULL};

spMaterial material = {0.4f, 0.6f};
spFilter filter = spFilterCollideAll;

spInt constraintCount = 0;
spInt shapeCount = 0;

spBody*
Body(spInt i)
{
    return shapes[i].body;
}

spShape*
Shape(spInt i)
{
    return shapes[i].shape;
}

spConstraint* 
Constraint(spInt i)
{
    return constraints[i];
}

spDemoShape* 
DemoShape(spInt i)
{
    return shapes+i;
}

spInt 
spCreateDemoSegment(spVector pos, spFloat angle, spVector pointA, spVector pointB, spFloat radius, spFloat mass, spColor color, spColor border, spBodyType type)
{
    shapes[shapeCount].body = spBodyNew(type);
    shapes[shapeCount].shape = spSegmentNew(pointA, pointB, radius, 0.0f);
    shapes[shapeCount].shape->material = material;
    spShapeSetFilter(shapes[shapeCount].shape, filter);
    spBodySetTransform(shapes[shapeCount].body, pos, angle);
    spBodyAddShape(shapes[shapeCount].body, shapes[shapeCount].shape);
    spWorldAddBody(&demo->world, shapes[shapeCount].body);
    shapes[shapeCount].color = color;
    shapes[shapeCount].border = border;

    shapeCount++;

    return shapeCount-1;
}

spInt 
spCreateDemoBox(spVector pos, spFloat angle, spVector size, spFloat mass, spColor color, spColor border, spBodyType type)
{
    spVector vertices[4];
    vertices[0] = spVectorConstruct(-size.x,-size.y); 
    vertices[1] = spVectorConstruct( size.x,-size.y);
    vertices[2] = spVectorConstruct( size.x, size.y);
    vertices[3] = spVectorConstruct(-size.x, size.y);

    shapes[shapeCount].body = spBodyNew(type);
    shapes[shapeCount].shape = spPolygonNew(vertices, 4, mass);
    shapes[shapeCount].shape->material = material;
    spShapeSetFilter(shapes[shapeCount].shape, filter);
    spBodySetTransform(shapes[shapeCount].body, pos, angle);
    spBodyAddShape(shapes[shapeCount].body, shapes[shapeCount].shape);
    spWorldAddBody(&demo->world, shapes[shapeCount].body);

    shapes[shapeCount].color = color;
    shapes[shapeCount].border = border;

    shapeCount++;

    return shapeCount-1;
}

spInt 
spCreateDemoCircle(spVector pos, spFloat radius, spFloat mass, spColor color, spColor border, spBodyType type)
{
    shapes[shapeCount].body = spBodyNew(type);
    shapes[shapeCount].shape = spCircleNew(spVectorConstruct(0.0f, 0.0f), radius, mass);
    shapes[shapeCount].shape->material = material;
    spBodySetTransform(shapes[shapeCount].body, pos, 0.0f);
    spBodyAddShape(shapes[shapeCount].body, shapes[shapeCount].shape);
    spWorldAddBody(&demo->world, shapes[shapeCount].body);

    shapes[shapeCount].color = color;
    shapes[shapeCount].border = border;

    shapeCount++;

    return shapeCount-1;
}

spConstraint* 
spAddConstraint(spConstraint* constraint)
{
    spWorldAddConstraint(&demo->world, constraint);
    constraints[constraintCount] = constraint;
    constraintCount++;
    return constraint;
}

void 
spDrawFactoryObjects()
{
    spWorld* world = &demo->world;

    for (spInt i = 0; i < shapeCount; ++i)
    {
        spDemoDrawShape(shapes[i].shape, shapes[i].color, shapes[i].border);
    }

    for (spInt i = 0; i < constraintCount; ++i)
    {
        spDemoDrawConstraint(constraints[i]);
    }
    if (demo->mouse.constraint != NULL && demo->mouse.shape != NULL)
    {
        spDemoDrawConstraint(demo->mouse.constraint);
    }
}
