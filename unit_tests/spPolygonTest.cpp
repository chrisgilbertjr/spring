
#include "spUnitTest.h"
#include "spPolygon.h"

SP_TEST(polygon)
{
    {
        SP_SUBTEST(spPolygonInit);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spPolygonAlloc);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spPolygonNew);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spPolygonFree);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spPolygonComputeCenterOfMass);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spPolygonComputeInertia);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spPolygonComputeBound);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spPolygonComputeMassData);
        SP_SUBTEST_RESULT();
    }
}

SP_TESTS(polygon_tests)
{
    SP_REG_TEST(polygon)
};