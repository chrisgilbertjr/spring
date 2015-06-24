
#include "spUnitTest.h"
#include "spCircle.h"
#include "spBody.h"

SP_TEST(circle)
{
    spFloat ra = 1.0f;
    spFloat rb = 2.0f;
    spFloat rc = 1.9182736f;
    spVector ca = spVector(0.0f, 0.0f);
    spVector cb = spVector(1.0f, 2.0f);
    spVector cc = spVector(1.1234567f, 3.9876543f);
    spCircleDef defa, defb, defc;

    defa.material = spMaterial(1.0f, 1.0f);
    defb.material = spMaterial(1.0f, 1.0f);
    defc.material = spMaterial(1.0f, 1.0f);
    defa.mass = 1.0f;
    defb.mass = 1.0f;
    defc.mass = 1.0f;

    defa.center = ca;
    defa.radius = ra;
    defb.center = cb;
    defb.radius = rb;
    defc.center = cc;
    defc.radius = rc;

    {
        SP_SUBTEST(spCircleInit);
        spCircle circlea;
        spCircle circleb;
        spBody body;

        spBodyInit(&body, SP_BODY_DYNAMIC);
        spCircleInit(&circlea, &body, defa);
        spCircleInit(&circleb, &body, defb);

        spMassData camd = circlea.base_class.mass_data;
        spMassData cbmd = circleb.base_class.mass_data;
        spMaterial cam  = circlea.base_class.material;
        spMaterial cbm  = circleb.base_class.material;
        spShapeType cat = circlea.base_class.type;
        spShapeType cbt = circleb.base_class.type;
        spBound cabd = circlea.base_class.bound;
        spBound cbbd = circleb.base_class.bound;
        spShape* casn = circlea.base_class.next;
        spShape* cbsn = circleb.base_class.next;
        spShape* casp = circlea.base_class.prev;
        spShape* cbsp = circleb.base_class.prev;
        spBody* cab  = circlea.base_class.body;
        spBody* cbb  = circleb.base_class.body;

        SP_ISEQ(cat, cbt);
        SP_ISEQ(cat, SP_SHAPE_CIRCLE);
        SP_ISEQ(cab, cbb);
        SP_ISEQ(cab, &body);
        SP_VECNEQ(camd.com, cbmd.com);
        SP_VECEQ(camd.com, spVectorZero());
        SP_FLTNEQ(camd.inertia, cbmd.inertia);
        SP_FLTEQ(camd.inertia, 0.5f);
        SP_FLTEQ(camd.mass, cbmd.mass);
        SP_FLTEQ(camd.mass, 1.0f);
        SP_FLTEQ(cam.friction, cbm.friction);
        SP_FLTEQ(cam.friction, 1.0f);
        SP_FLTEQ(cam.restitution, cbm.restitution);
        SP_FLTEQ(cam.restitution, 1.0f);
        SP_VECNEQ(cabd.center, cbbd.center);
        SP_VECEQ(cabd.center, spVectorZero());
        SP_VECEQ(cabd.half_width, spVector(ra, ra));
        SP_FLTNEQ(cabd.radius, cbbd.radius);
        SP_FLTEQ(cabd.radius, ra);
        SP_VECEQ(circlea.center, ca);
        SP_VECEQ(circleb.center, cb);
        SP_FLTEQ(circlea.radius, ra);
        SP_FLTEQ(circleb.radius, rb);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCircleAlloc);
        spCircle* circle = spCircleAlloc();
        circle->center = ca;
        circle->radius = ra;
        SP_ISTRUE(circle != NULL);
        SP_VECEQ(circle->center, ca);
        SP_FLTEQ(circle->radius, ra);
        spFree(circle);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCircleNew);
        spBody body;
        spBodyInit(&body, SP_BODY_DYNAMIC);
        spShape* shape = spCircleNew(&body, defa);
        spCircle* circle = (spCircle*)shape;
        SP_ISTRUE(circle != NULL);
        SP_VECEQ(circle->center, ca);
        SP_FLTEQ(circle->radius, ra);
        spFree(circle);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCircleFree);
        spCircle* circle = spCircleAlloc();
        SP_ISFALSE(circle == NULL);
        spFree(circle);
        SP_ISTRUE(circle == NULL);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCircleComputeInertia);
        spCircle circlea;
        spCircle circleb;
        spFloat massa = 1.0f;
        spFloat massb = 2.0f;
        spFloat inertiaa = 1.0f * 1.0f * 1.0f / 2.0f;
        spFloat inertiab = 2.0f * 2.0f * 2.0f / 2.0f;
        circlea.radius = 1.0f;
        circleb.radius = 2.0f;

        SP_FLTEQ(spCircleComputeInertia(&circlea, massa), inertiaa);
        SP_FLTEQ(spCircleComputeInertia(&circleb, massb), inertiab);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCircleComputeBound);
        spCircle circlea;
        spCircle circleb;
        spBound bounda;
        spBound boundb;
        spBody body;
        spVector hwa = spVector(ra, ra);
        spVector hwb = spVector(rb, rb);

        spBodyInit(&body, SP_BODY_DYNAMIC);
        spCircleInit(&circlea, &body, defa);
        spCircleInit(&circleb, &body, defb);
        spCircleComputeBound(&circlea, &bounda);
        spCircleComputeBound(&circleb, &boundb);

        SP_FLTEQ(bounda.radius, ra);
        SP_VECEQ(bounda.center, ca);
        SP_VECEQ(bounda.half_width, hwa);
        SP_FLTEQ(boundb.radius, rb);
        SP_VECEQ(boundb.center, cb);
        SP_VECEQ(boundb.half_width, hwb);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCircleComputeMassData);
        spMassData mass_data;
        spCircle circlea;
        spBody body;
        spCircleInit(&circlea, &body, defa);
        spVector com = spVectorZero();
        spFloat mass = 2.0f;
        spFloat inertiaa = mass * circlea.radius * circlea.radius / 2.0f;
        spCircleComputeMassData(&circlea, &mass_data, mass);

        SP_FLTEQ(mass_data.mass, mass);
        SP_VECEQ(mass_data.com, com);
        SP_FLTEQ(mass_data.inertia, inertiaa);
        SP_SUBTEST_RESULT();
    }
}

SP_TESTS(circle_tests)
{
    SP_REG_TEST(circle)
};
