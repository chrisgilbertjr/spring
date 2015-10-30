
#ifndef SP_DEBUG_DRAW_H
#define SP_DEBUG_DRAW_H

#include "spContact.h"
#include "spSegment.h"
#include "spPolygon.h"
#include "spCircle.h"
#include "spBound.h"

#include <GL\glew.h>

struct spDebugDraw
{
};

struct spColor
{
    spFloat r;
    spFloat g;
    spFloat b;
    spFloat a;
};

INLINE spColor _spColor(spFloat r, spFloat g, spFloat b, spFloat a)
{
    return { r, g, b, a };
}

#define spPurple(a) _spColor(1.0f, 0.0f, 1.0f, a)
#define spYellow(a) _spColor(1.0f, 1.0f, 0.0f, a)
#define spGreen(a)  _spColor(0.0f, 1.0f, 0.0f, a)
#define spBlack(a)  _spColor(0.0f, 0.0f, 0.0f, a)
#define spWhite(a)  _spColor(1.0f, 1.0f, 1.0f, a)
#define spBlue(a)   _spColor(0.0f, 0.0f, 1.0f, a)
#define spRed(a)    _spColor(1.0f, 0.0f, 0.0f, a)

void spDebugDrawPoint(const spVector pos, const spColor& color);

void spDebugDrawPoint(const spFloat x, const spFloat y, const spColor color);

void spDebugDrawLine(const spVector a, const spVector b, const spColor color);

void spDebugDrawFatLine(const spVector a, const spVector b, spFloat size, const spColor color);

void spDebugDrawTriangle(const spVector a, const spVector& b, const spVector& c, const spColor& color);

void spDebugDrawCircle(spDebugDraw* draw, const spCircle* circle, const spTransform xf);

void spDebugDrawPolygon(spDebugDraw* draw, const spPolygon* polygon, const spTransform xf);

void spDebugDrawSegment(spDebugDraw* draw, const spSegment* segment);

void spDebugDrawContact(spDebugDraw* draw, spContact* contact, const spTransform xf);

void spDebugDrawBound(spDebugDraw* draw, spBound* bound, const spTransform xf);

#endif