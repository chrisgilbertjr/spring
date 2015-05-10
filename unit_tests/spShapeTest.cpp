
#include "spUnitTest.h"
#include "spShape.h"

SP_TEST(mass_data)
{
    {
        SP_SUBTEST(spMassDataInit);
        SP_SUBTEST_RESULT();
    }
}

SP_TEST(material)
{
    {
        SP_SUBTEST(spMaterial);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spMaterialComputeFriction);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spMaterialComputeRestitution);
        SP_SUBTEST_RESULT();
    }
}

SP_TEST(shape)
{
    {
        SP_SUBTEST(spShapeInit);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spShapeLessThan);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spShapeAdd);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spShapeRemove);
        SP_SUBTEST_RESULT();
    }
}

SP_TESTS(shape_tests)
{
    SP_REG_TEST(mass_data),
    SP_REG_TEST(material),
    SP_REG_TEST(shape)
};
