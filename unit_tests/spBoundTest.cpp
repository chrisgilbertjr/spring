
#include "spUnitTest.h"
#include "spBound.h"

SP_TEST(bound)
{
    {
        SP_SUBTEST(spBoundInit);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spBound);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spBoundSet);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spBoundBoxPerimeter);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spBoundBoxArea);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spBoundBoxOverlap);
        SP_SUBTEST_RESULT();
    }
}

SP_TESTS(bound_tests)
{
    SP_REG_TEST(bound)
};