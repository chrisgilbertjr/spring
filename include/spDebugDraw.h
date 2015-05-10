
#ifndef SP_DEBUG_DRAW_H
#define SP_DEBUG_DRAW_H

#include "spContact.h"
#include "spPolygon.h"
#include "spCircle.h"
#include "spBound.h"

#include <GL\glew.h>

struct spDebugDraw
{
};

void spDebugDrawCircle(spDebugDraw* draw, const spCircle* circle, const spTransform& xf);

void spDebugDrawPolygon(spDebugDraw* draw, const spPolygon* polygon, const spTransform& xf);

void spDebugDrawContact(spDebugDraw* draw, spContact* contact, const spTransform& xf);

void spDebugDrawBound(spDebugDraw* draw, spBound* bound, const spTransform& xf);

#endif