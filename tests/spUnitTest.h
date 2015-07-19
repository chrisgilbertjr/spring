
#ifndef SP_UNIT_TEST_H
#define SP_UNIT_TEST_H

#include "spCore.h"
#include <string.h>

typedef void (*test_func)(); ///< test function pointer
typedef spBool spResult; ///< test result

/// represents a unit test
struct spUnitTest
{
    test_func function; ///< the function pointer
    spInt8 name[32]; ///< the name of the unit test
};

const spInt SP_MAX_TESTS = 4; ///< the max number of unit tests for a test category
extern spUnitTest bound_tests[SP_MAX_TESTS];
extern spUnitTest chain_tests[SP_MAX_TESTS];
extern spUnitTest circle_tests[SP_MAX_TESTS];
extern spUnitTest math_tests[SP_MAX_TESTS]; ///< math unit tests
extern spUnitTest polygon_tests[SP_MAX_TESTS];
extern spUnitTest shape_tests[SP_MAX_TESTS];

/// logs that a unit test category is starting
inline void spStartTest(const spInt8* test_name)
{
    spLog("%s test\n", test_name);
}

/// logs that a subtest is starting
inline void spStartSubtest(const spInt8* subtest_name)
{
    spLog(" %s test\n", subtest_name);
}

/// logs a subtests results
inline void spLogSubtestResult(spResult result, const spInt8* subtest_name)
{
    const spInt8* test_result[2] = { "*FAIL*", " PASS " };
    const spInt8* result_str = " result";

    spInt8 buffer[72] = { 0 };
    spMemset(buffer, '_', 71);
    strncpy(buffer, subtest_name, strlen(subtest_name));
    strncpy(buffer + strlen(subtest_name), result_str, strlen(result_str));
    spLog(" %s %s\n\n", buffer, test_result[result]);
}

/// logs an equality check during a test
inline void spLogEquality(spResult result, const spInt8* test, const spInt8* a, const spInt8* b)
{
    const spInt8* test_result[2] = { " fail ", " pass " };
    spInt8 buffer[70] = { 0 };
    spMemset(buffer, '.', 69);

    spInt8* mem = buffer;
    strncpy(mem, test, strlen(test));  mem += strlen(test);
    strncpy(mem, "(",  strlen("("));   mem += strlen("(");
    strncpy(mem, a,    strlen(a));     mem += strlen(a);
    if (b != NULL)
    {
        strncpy(mem, " ",  strlen(" "));   mem += strlen(" ");
        strncpy(mem, b,    strlen(b));     mem += strlen(b);
    }
    strncpy(mem, ")",  strlen(")"));   mem += strlen(")");
    spLog("   %s %s\n", buffer, test_result[result]);
}

/// registers a test so it can be ran
inline spUnitTest spRegisterTest(test_func function, const spInt8* name)
{
    spUnitTest t;
    t.function = function;
    strcpy(t.name, name);
    return t;
}

/// turns a macro param into a c string
#define SP_STRINGIFY(str) #str

/// defines a subtest within a test
#define SP_SUBTEST(subtest)  \
const spInt8* subtest_name = SP_STRINGIFY(subtest); \
spResult subtest_result = spTrue; \
spStartSubtest(subtest_name); \

/// logs a subtests result
#define SP_SUBTEST_RESULT() spLogSubtestResult(subtest_result, subtest_name);

/// check if two floating point numbers are equal
#define SP_FLTEQ(a, b) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    const spInt8* b_str = SP_STRINGIFY(b); \
    spResult equality_result = spFalse; \
    equality_result = spAlmostEqual(a, b, SP_EPSILON); \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_FLTEQ", a_str, b_str); \
} \

/// logs if two floating point numbers are not equal
#define SP_FLTNEQ(a, b) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    const spInt8* b_str = SP_STRINGIFY(b); \
    spResult equality_result = spFalse; \
    equality_result = !spAlmostEqual(a, b, SP_EPSILON); \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_FLTNEQ", a_str, b_str); \
} \

/// logs if a boolean is true
#define SP_ISTRUE(a) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    spResult equality_result = spFalse; \
    equality_result = (a == spTrue); \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_ISTRUE", a_str, 0); \
} \

/// logs if a boolean is false
#define SP_ISFALSE(a) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    spResult equality_result = spFalse; \
    equality_result = (a == spFalse); \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_ISFALSE", a_str, 0); \
} \

/// logs if two vectors are equal
#define SP_VECEQ(a, b) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    const spInt8* b_str = SP_STRINGIFY(b); \
    spResult equality_result = spFalse; \
    equality_result = spAlmostEqual(a.x, b.x, SP_EPSILON) && spAlmostEqual(a.y, b.y, SP_EPSILON); \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_VECEQ", a_str, b_str); \
} \

/// logs if two vectors are not equal
#define SP_VECNEQ(a, b) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    const spInt8* b_str = SP_STRINGIFY(b); \
    spResult equality_result = spFalse; \
    equality_result = !spAlmostEqual(a.x, b.x, SP_EPSILON) || !spAlmostEqual(a.y, b.y, SP_EPSILON); \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_VECNEQ", a_str, b_str); \
} \

/// logs if two pointers are equal
#define SP_ISEQ(a, b) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    const spInt8* b_str = SP_STRINGIFY(b); \
    spResult equality_result = spFalse; \
    equality_result = a == b; \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_ISEQ", a_str, b_str); \
} \

/// logs if two pointers are equal
#define SP_ISNEQ(a, b) \
{ \
    const spInt8* a_str = SP_STRINGIFY(a); \
    const spInt8* b_str = SP_STRINGIFY(b); \
    spResult equality_result = spFalse; \
    equality_result = a != b; \
    if (subtest_result == spTrue) \
    { \
        subtest_result = equality_result; \
    } \
    spLogEquality(equality_result, "SP_ISNEQ", a_str, b_str); \
} \

/// registers a test to be run
#define SP_REG_TEST(test) spRegisterTest(test, SP_STRINGIFY(test))

/// registered tests inside of this macro so they can be run
#define SP_TESTS(test_name) spUnitTest test_name[SP_MAX_TESTS] = 

/// defines a new test function
#define SP_TEST(test_name) void test_name()

/// runs all tests of type test_name
#define SP_RUN_TESTS(test_name) \
    for (spInt i = 0; test_name[i].function != 0; ++i) \
    { \
        spStartTest(test_name[i].name); \
        test_name[i].function(); \
    } \


#endif