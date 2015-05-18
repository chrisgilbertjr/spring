
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

struct spColor
{
    spFloat r;
    spFloat g;
    spFloat b;
};

inline spColor _spColor(spFloat r, spFloat g, spFloat b)
{
    return { r, g, b };
}

#define spPurple() _spColor(1.0f, 0.0f, 1.0f)
#define spYellow() _spColor(1.0f, 1.0f, 0.0f)
#define spGreen()  _spColor(0.0f, 1.0f, 0.0f)
#define spBlack()  _spColor(0.0f, 0.0f, 0.0f)
#define spWhite()  _spColor(1.0f, 1.0f, 1.0f)
#define spBlue()   _spColor(0.0f, 0.0f, 1.0f)
#define spRed()    _spColor(1.0f, 0.0f, 0.0f)

void spDebugDrawPoint(const spVector& pos, const spColor& color);

void spDebugDrawPoint(const spFloat x, const spFloat y, const spColor& color);

void spDebugDrawLine(const spVector& a, const spVector b, const spColor& color);

void spDebugDrawTriangle(const spVector& a, const spVector& b, const spVector& c, const spColor& color);

void spDebugDrawCircle(spDebugDraw* draw, const spCircle* circle, const spTransform& xf);

void spDebugDrawPolygon(spDebugDraw* draw, const spPolygon* polygon, const spTransform& xf);

void spDebugDrawContact(spDebugDraw* draw, spContact* contact, const spTransform& xf);

void spDebugDrawBound(spDebugDraw* draw, spBound* bound, const spTransform& xf);

#endif