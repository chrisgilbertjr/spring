
#ifndef SPRING_H
#define SPRING_H

#include "spWorld.h"

/// create a new rigid body and insert it into the world
spBody* spCreateBody(spWorld* world, spBodyType type);

/// remove a rigid body from the world and destroy it
void spDestroyBody(spWorld* world, spBody* body);

/// adds a circle to a body
void spAddCircle(spCircle* circle, spBody* body);

/// removes a circle from a body
void spRemoveCircle(spCircle* circle);

/// create a new circle but does not add it to a body
spCircle* spCreateCircle(const spCircleDef& def);

/// create a new circle
spCircle* spCreateCircle(const spVector& center, spFloat radius, const spMaterial& material, spFloat mass);

/// create a new circle and add it to a body
spCircle* spCreateCircle(spBody* body, const spCircleDef& def);

/// create a new circle and add it to a body
spCircle* spCreateCircle(spBody* body, const spVector& center, spFloat radius, const spMaterial& material, spFloat mass);

/// destroy a circle and free its memory
void spDestroyCircle(spCircle* circle);

/// adds a polygon to a body
void spAddPolygon(spPolygon* poly, spBody* body);

/// removes a polygon from a body
void spRemovePolygon(spPolygon* poly);

/// create a new polygon with a polygon definition
spPolygon* spCreatePolygon(const spPolygonDef& def);

/// create a new polygon
spPolygon* spCreatePolygon(spVector* vertices, spInt count, const spMaterial& material, spFloat mass);

/// create a new polygon and add it to a body
spPolygon* spCreatePolygon(spBody* body, const spPolygonDef& def);

/// create a new polygon and add it to a body
spPolygon* spCreatePolygon(spBody* body, spVector* vertices, spInt count, const spMaterial& material, spFloat mass);

/// destroy a polygon and free its memory
void spDestroyPolygon(spPolygon* poly);

#endif