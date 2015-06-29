
#include "spUnitTest.h"
#include "spMath.h"

/// all math constants are computed using GNU Octave, version 3.8.2
SP_TEST(vector)
{
    spFloat a = 0.123456f;
    spFloat b = 1.234567f;
    spFloat c = 2.345678f;
    spFloat d = 3.456789f;

    spVector aa; aa.x = a; aa.y = a;
    spVector ab; ab.x = a; ab.y = b;
    spVector ba; ba.x = b; ba.y = a;
    spVector bb; bb.x = b; bb.y = b;
    spVector cc; cc.x = c; cc.y = c;
    spVector dd; dd.x = d; dd.y = d;

    {
        SP_SUBTEST(spVector);
        spVector ra = _spVector(a, b);
        spVector rb = _spVector(0.123456f, 1.234567f);

        SP_VECEQ(ra, rb);
        SP_VECEQ(ra, ab);
        SP_VECEQ(rb, ab);
        SP_VECNEQ(ra, ba);
        SP_VECNEQ(rb, ba);
        SP_VECNEQ(ab, ba);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spVectorSet);
        spVector ra; ra.x = a; ra.y = b;
        spVector rb;
        spVector rc;
        spVectorSet(&rb, a, b);
        spVectorSet(&rc, b, a);

        SP_VECEQ(ra, rb);
        SP_VECNEQ(ra, rc);
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spEqual);
        spVector ra; ra.x = a; ra.y = b;
        spVector rb; rb.x = a; rb.y = b;
        spVector rc; rc.x = a; rc.y = a;

        SP_ISTRUE(spEqual(ra, rb));
        SP_ISTRUE(spEqual(rb, ra));
        SP_ISFALSE(spEqual(ra, rc));
        SP_ISFALSE(spEqual(rc, ra));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spLessThan);
        SP_ISTRUE(spLessThan(aa, dd));
        SP_ISTRUE(spLessThan(bb, dd));
        SP_ISTRUE(spLessThan(cc, dd));
        SP_ISFALSE(spLessThan(dd, dd));
        SP_ISFALSE(spLessThan(dd, cc));
        SP_ISFALSE(spLessThan(dd, bb));
        SP_ISFALSE(spLessThan(dd, aa));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spDot);
        spFloat dot_aa_aa =  0.0304828f;
        spFloat dot_aa_bb =  0.3048294f;
        spFloat dot_aa_cc =  0.5791760f;
        spFloat dot_aa_dd =  0.8535227f;

        SP_FLTEQ(dot_aa_aa, spDot(aa, aa));
        SP_FLTEQ(dot_aa_bb, spDot(aa, bb));
        SP_FLTEQ(dot_aa_cc, spDot(aa, cc));
        SP_FLTEQ(dot_aa_dd, spDot(aa, dd));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spLength);
        spFloat length_aa =  0.1745931f;
        spFloat length_bb =  1.7459414f;
        spFloat length_cc =  3.3172896f;
        spFloat length_dd =  4.8886378f;

        SP_FLTEQ(length_aa, spLength(aa));
        SP_FLTEQ(length_bb, spLength(bb));
        SP_FLTEQ(length_cc, spLength(cc));
        SP_FLTEQ(length_dd, spLength(dd));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spLengthSquared);
        spFloat lensqrAa =  0.0304827f;
        spFloat lensqrBb =  3.0483113f;
        spFloat lensqr_cc = 11.0044105f;
        spFloat lensqr_dd = 23.8987801f;

        SP_FLTEQ(lensqrAa, spLengthSquared(aa));
        SP_FLTEQ(lensqrBb, spLengthSquared(bb));
        SP_FLTEQ(lensqr_cc, spLengthSquared(cc));
        SP_FLTEQ(lensqr_dd, spLengthSquared(dd));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spSkew);
        spVector skew_aa; skew_aa.x = -aa.y; skew_aa.y = aa.x;

        SP_VECEQ(skew_aa, spSkew(aa));
        SP_VECNEQ(spSkew(aa), spSkewT(aa));
        SP_VECNEQ(aa, spSkew(aa));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spSkewT);
        spVector skewT_aa; skewT_aa.x = aa.y; skewT_aa.y = -aa.x;

        SP_VECEQ(skewT_aa, spSkewT(aa));
        SP_VECNEQ(spSkew(aa), spSkewT(aa));
        SP_VECNEQ(aa, spSkewT(aa));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spAdd);
        spVector add_aa_aa;
        spVector add_aa_bb;
        spVector add_bb_dd;

        add_aa_aa.x = add_aa_aa.y = 0.2469119f;
        add_aa_bb.x = add_aa_bb.y = 1.3580229f;
        add_bb_dd.x = add_bb_dd.y = 4.6913559f;

        SP_VECEQ(add_aa_aa, spAdd(aa, aa));
        SP_VECEQ(add_aa_bb, spAdd(aa, bb));
        SP_VECEQ(add_bb_dd, spAdd(bb, dd));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spSub);
        spVector sub_aa_bb;
        spVector sub_cc_bb;
        spVector sub_dd_aa;

        sub_aa_bb.x = sub_aa_bb.y = -1.1111109f;
        sub_cc_bb.x = sub_cc_bb.y =  1.1111109f;
        sub_dd_aa.x = sub_dd_aa.y =  3.3333330f;

        SP_VECEQ(sub_aa_bb, spSub(aa, bb));
        SP_VECEQ(sub_cc_bb, spSub(cc, bb));
        SP_VECEQ(sub_dd_aa, spSub(dd, aa));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spMult_vec_float);
        spVector mult_aa_d;
        spVector mult_bb_d;
        spVector mult_cc_d;

        mult_aa_d.x = mult_aa_d.y = 0.4267613f;
        mult_bb_d.x = mult_bb_d.y = 4.2676376f;
        mult_cc_d.x = mult_cc_d.y = 8.1085139f;

        SP_VECEQ(mult_aa_d, spMult(aa, d));
        SP_VECEQ(mult_bb_d, spMult(bb, d));
        SP_VECEQ(mult_cc_d, spMult(cc, d));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spMult_float_vec);
        spVector mult_aa_d;
        spVector mult_bb_d;
        spVector mult_cc_d;

        mult_aa_d.x = mult_aa_d.y = 0.4267613f;
        mult_bb_d.x = mult_bb_d.y = 4.2676376f;
        mult_cc_d.x = mult_cc_d.y = 8.1085139f;

        SP_VECEQ(mult_aa_d, spMult(d, aa));
        SP_VECEQ(mult_bb_d, spMult(d, bb));
        SP_VECEQ(mult_cc_d, spMult(d, cc));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spNormalize);
        spVector norm_aa = aa;
        spVector norm_ab = ab;
        spVector norm_cc = cc;

        spNormalize(&norm_aa);
        spNormalize(&norm_ab);
        spNormalize(&norm_cc);

        SP_FLTEQ(1.0f, spLength(norm_aa));
        SP_FLTEQ(1.0f, spLength(norm_ab));
        SP_FLTEQ(1.0f, spLength(norm_cc));
        SP_SUBTEST_RESULT()
    }

    {
        SP_SUBTEST(spDistanceSquared);
        spFloat distsqrAa_bb = 2.4691353f;
        spFloat distsqr_dd_cc = 2.4691353f;

        SP_FLTEQ(distsqrAa_bb, spDistanceSquared(aa, bb));
        SP_FLTEQ(distsqr_dd_cc, spDistanceSquared(dd, cc));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spDistance);
        spFloat dist_aa_bb = 1.5713482f;
        spFloat dist_dd_cc = 1.5713482f;

        SP_FLTEQ(dist_aa_bb, spDistance(aa, bb));
        SP_FLTEQ(dist_dd_cc, spDistance(dd, cc));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCross_vec_vec);
        /// GNU octave does not have 2d cross functions
        spFloat cross_aa_bb = a * b - a * b;
        spFloat cross_ab_ba = a * a - b * b;

        SP_FLTEQ(cross_aa_bb, spCross(aa, bb));
        SP_FLTEQ(cross_ab_ba, spCross(ab, ba));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spCross_vec_float);
        /// GNU octave does not have 2d cross functions
        spVector cross_aa_a = spVector(a * a, -a * a);
        spVector cross_aa_b = spVector(b * a, -b * a);
        spVector cross_ba_d = spVector(d * a, -d * b);

        SP_VECEQ(cross_aa_a, spCross(aa, a));
        SP_VECEQ(cross_aa_b, spCross(aa, b));
        SP_VECEQ(cross_ba_d, spCross(ba, d));
        SP_SUBTEST_RESULT();
    }

    {
        SP_SUBTEST(spNegate);
        spVector neg_aa = aa;
        spVector neg_bb = bb;

        spNegate(&neg_aa);
        spNegate(&neg_bb);

        SP_VECEQ(neg_aa, spVector(-a, -a));
        SP_VECEQ(neg_bb, spVector(-b, -b));
        SP_VECNEQ(neg_aa, aa);
        SP_SUBTEST_RESULT();
    }
}

SP_TEST(matrix)
{
}

SP_TEST(rotation)
{
}

SP_TEST(transform)
{
}

SP_TESTS(math_tests)
{
    SP_REG_TEST(vector),
    SP_REG_TEST(matrix),
    SP_REG_TEST(rotation),
    SP_REG_TEST(transform)
};
