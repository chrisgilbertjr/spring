
#include "spDebugDraw.h"
#include "spBody.h"

void 
spDebugDrawPoint(const spVector& pos, const spColor& color)
{
    glPointSize(8.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(color.r, color.g, color.b, color.a);
    glBegin(GL_POINTS);
    glVertex2f(pos.x, pos.y);
    glEnd();
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
}

void 
spDebugDrawPoint(const spFloat x, const spFloat y, const spColor& color)
{
    spDebugDrawPoint(spVectorConstruct(x, y), color);
}

void 
spDebugDrawLine(const spVector& a, const spVector b, const spColor& color)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(color.r, color.g, color.b, color.a);
    glBegin(GL_LINES);
    glVertex2f(a.x, a.y);
    glVertex2f(b.x, b.y);
    glEnd();
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
}

void 
spDebugDrawFatLine(const spVector& a, const spVector b, spFloat size, const spColor& color)
{
    glLineWidth(size);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(color.r, color.g, color.b, color.a);
    glBegin(GL_LINES);
    glVertex2f(a.x, a.y);
    glVertex2f(b.x, b.y);
    glEnd();
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glLineWidth(1.0f);
}

void 
spDebugDrawTriangle(const spVector& a, const spVector& b, const spVector& c, const spColor& color)
{
    glColor4f(color.r, color.g, color.b, color.a);
    glBegin(GL_TRIANGLES);
    glVertex2f(a.x, a.y);
    glVertex2f(b.x, b.y);
    glVertex2f(c.x, c.y);
    glEnd();
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
}

void
spDebugDrawCircle(spDebugDraw* draw, const spCircle* circle, const spTransform& xf)
{
    const spFloat iters = 32.f;

    spFloat  radius = circle->radius;
    spVector center = spMultXformVec(xf, circle->center);

    glPushMatrix();
    glLoadIdentity();
    glTranslatef(center.x, center.y, 0.0f);
    glRotatef(spRotationGetAngleDeg(xf.q), 0.0f, 0.0f, 1.0f);
    glScalef(radius, radius, 0.0f);

    glBegin(GL_LINES);
    glVertex2f(0.f, 1.f);
    glVertex2f(0.f, 0.f);
    glEnd();

    glBegin(GL_LINE_LOOP);
    for (spFloat i = 0.f; i < iters; i += 1.0f)
    {
        spFloat s = i / iters * SP_PI * 2.0f;
        spFloat c = i / iters * SP_PI * 2.0f;
        glVertex2f(spsin(s), spcos(s));
    }
    glEnd();
    glPopMatrix();
}

void spDebugDrawPolygon(spDebugDraw* draw, const spPolygon* polygon, const spTransform& xf)
{
    spInt    count = polygon->count;
    spEdge*  edges = polygon->edges;
    spFloat  angle = spRotationGetAngleDeg(xf.q);
    spVector pos = xf.p;

    glPushMatrix();
    glLoadIdentity();
    glTranslatef(pos.x, pos.y, 0.0f);
    glRotatef(angle, 0.0f, 0.0f, 1.0f);

    glBegin(GL_LINES);
    for (spInt i = 0; i < count; ++i)
    {
        spEdge* edge0 = edges + i;
        spEdge* edge1 = edges + ((i + 1) % count);
        spVector v0 = edge0->vertex;
        spVector v1 = edge1->vertex;
        glVertex2f(v0.x, v0.y);
        glVertex2f(v1.x, v1.y);
    }
    glEnd();
    glPopMatrix();
}

void 
spDebugDrawSegment(spDebugDraw* draw, const spSegment* segment)
{
    spBody* body = segment->shape.body;
    spTransform* xf = &body->xf;
    spVector a = spMultXformVec(*xf, segment->pointA);
    spVector b = spMultXformVec(*xf, segment->pointB);
    spVector n = segment->normal;
    spFloat  r = segment->radius;

     glBegin(GL_LINE_LOOP);
     spFloat iters = 32.0f;
     for (spFloat i = 0.f; i < iters; i += 1.0f)
     {
         spFloat s = i / iters * SP_PI * 2.0f;
         spFloat c = i / iters * SP_PI * 2.0f;
     }
     glEnd();

    spVector c = spNormal(spSkewT(n));
    spVector h = spMultVecFlt(c, segment->radius);
    spVector v = spMultVecFlt(n, segment->radius);
    spVector p0 = spAddVecs(b, h); 
    spVector p1 = spSubVecs(a, h);
    spDebugDrawLine(p0, p1, spRed(1.0f)); p0 = spAddVecs(a, v); p1 = spSubVecs(a, v);
    spDebugDrawLine(p0, p1, spRed(1.0f)); p0 = spAddVecs(b, v); p1 = spSubVecs(b, v);
    spDebugDrawLine(p0, p1, spRed(1.0f));
    spDebugDrawLine(a, b, spGreen(1.0f));
}

void 
spDebugDrawContact(spDebugDraw* draw, spContact* contact, const spTransform& xf)
{
    glPointSize(8.0f);
    glPushMatrix();
 
     spVector normal = contact->normal;
     for (spInt i = 0; i < contact->count; ++i)
     {
         glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
         spVector pos  = spVectorZero();
         spVector norm = spAddVecs(pos, normal);
         glBegin(GL_POINTS);
         glVertex2f(pos.x, pos.y);
         glEnd();

         glBegin(GL_LINES);
         glVertex2f(pos.x, pos.y);
         glVertex2f(norm.x, norm.y);
         glEnd();
         glColor3f(1.0f, 1.0f, 1.0f);
     }
    glPopMatrix();
}

void 
spDebugDrawBound(spDebugDraw* draw, spBound* bound, const spTransform& xf)
{
    spVector c  = spMultXformVec(xf, bound->center);
    spVector hw = bound->halfWidth;
    spVector p0 = spAddVecs(c, spVectorConstruct(-hw.x,-hw.y));
    spVector p1 = spAddVecs(c, spVectorConstruct( hw.x,-hw.y));
    spVector p2 = spAddVecs(c, spVectorConstruct( hw.x, hw.y));
    spVector p3 = spAddVecs(c, spVectorConstruct(-hw.x, hw.y));
    spFloat alpha = 0.1f;

    glPushMatrix();
     glEnable(GL_BLEND);
     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
     glBegin(GL_LINES);

     glColor4f(1.0f, 1.0f, 1.0f, alpha);
     glColor4f(1.0f, 1.0f, 1.0f, 0.05f);
     glVertex2f(p0.x, p0.y);
     glVertex2f(p1.x, p1.y);

     glVertex2f(p1.x, p1.y);
     glVertex2f(p2.x, p2.y);

     glVertex2f(p2.x, p2.y);
     glVertex2f(p3.x, p3.y);

     glVertex2f(p3.x, p3.y);
     glVertex2f(p0.x, p0.y);
     glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

     glEnd();
     //glDisable(GL_BLEND);
    glPopMatrix();
}