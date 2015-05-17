
#ifndef SPRING_H
#define SPRING_H

#include "spWorld.h"

/// create a new rigid body and insert it into the world
spBody* spCreateBody(spWorld* world, spBodyType type);

/// remove a rigid body from the world and destroy it
void spDestroyBody(spWorld* world, spBody* body);

/// create a new circle and add it to a body
spCircle* spCreateCircle(spBody* body, const spCircleDef& def);

/// destroy a circle and free its memory
void spDestroyCircle(spCircle* circle);

/// create a new polygon and add it to a body
spPolygon* spCreatePolygon(spBody* body, const spPolygonDef& def);

/// destroy a polygon and free its memory
void spDestroyPolygon(spPolygon* poly);

#endif