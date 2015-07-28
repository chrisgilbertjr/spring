
#ifndef SP_DEMO_FACTORY_H
#define SP_DEMO_FACTORY_H

#include "spPolygon.h"
#include "spCircle.h"
#include "spDemo.h"

#define spCreateKinematicSegment(p, a, A, B, r, m, c, b) spCreateDemoSegment(p, a, A, B, r, m, c, b, SP_BODY_KINEMATIC)
#define spCreateStaticSegment(p, a, A, B, r, m, c, b) spCreateDemoSegment(p, a, A, B, r, m, c, b, SP_BODY_STATIC)
#define spCreateSegment(p, a, A, B, r, m, c, b) spCreateDemoSegment(p, a, A, B, r, m, c, b, SP_BODY_DYNAMIC)

#define spCreateKinematicBox(p, a, s, m, c, b) spCreateDemoBox(p, a, s, m, c, b, SP_BODY_KINEMATIC)
#define spCreateStaticBox(p, a, s, m, c, b) spCreateDemoBox(p, a, s, m, c, b, SP_BODY_STATIC)
#define spCreateBox(p, a, s, m, c, b) spCreateDemoBox(p, a, s, m, c, b, SP_BODY_DYNAMIC)
#define spCreateDynamicBox(p, a, s, m, c, b) spCreateDemoBox(p, a, s, m, c, b, SP_BODY_DYNAMIC)

#define spCreateKinematicCircle(p, r, m, c, b) spCreateDemoCircle(p, r, m, c, b, SP_BODY_KINEMATIC)
#define spCreateStaticCircle(p, r, m, c, b) spCreateDemoCircle(p, r, m, c, b, SP_BODY_STATIC)
#define spCreateDynamicCircle(p, r, m, c, b) spCreateDemoCircle(p, r, m, c, b, SP_BODY_DYNAMIC)

struct spDemoShape { spShape* shape; spBody* body; spColor color, border; };

extern spConstraint* constraints[100];
extern spDemoShape shapes[100];

extern spMaterial material;
extern spFilter filter;

extern spInt constraintCount;
extern spInt shapeCount;

spBody* Body(spInt i);
spShape* Shape(spInt i);
spConstraint* Constraint(spInt i);
spDemoShape* DemoShape(spInt i);

spInt spCreateDemoSegment(spVector pos, spFloat angle, spVector pointA, spVector pointB, spFloat radius, spFloat mass, spColor color, spColor border, spBodyType type);
spInt spCreateDemoBox(spVector pos, spFloat angle, spVector size, spFloat mass, spColor color, spColor border, spBodyType type);
spInt spCreateDemoCircle(spVector pos, spFloat radius, spFloat mass, spColor color, spColor border, spBodyType type);
spConstraint* spAddConstraint(spConstraint* constraint);
void spDrawFactoryObjects();

#endif