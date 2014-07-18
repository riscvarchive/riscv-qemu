#include "exec/def-helper.h"

DEF_HELPER_3(raise_exception_err, noreturn, env, i32, int)
DEF_HELPER_2(raise_exception, noreturn, env, i32)

DEF_HELPER_3(mulhsu, tl, env, tl, tl)
//DEF_HELPER_2(riscv_exception, void, env, int)

//DEF_HELPER_1(read_count, tl, env)

DEF_HELPER_3(csrrw, tl, env, tl, tl)
DEF_HELPER_3(csrrs, tl, env, tl, tl)
DEF_HELPER_3(csrrc, tl, env, tl, tl)

DEF_HELPER_4(fadd_s, tl, env, tl, tl, tl)
DEF_HELPER_4(fsub_s, tl, env, tl, tl, tl)
DEF_HELPER_4(fmul_s, tl, env, tl, tl, tl)
DEF_HELPER_4(fdiv_s, tl, env, tl, tl, tl)
DEF_HELPER_3(fsgnj_s, tl, env, tl, tl)
DEF_HELPER_3(fsgnjn_s, tl, env, tl, tl)
DEF_HELPER_3(fsgnjx_s, tl, env, tl, tl)
DEF_HELPER_3(fmin_s, tl, env, tl, tl)
DEF_HELPER_3(fmax_s, tl, env, tl, tl)
DEF_HELPER_3(fsqrt_s, tl, env, tl, tl)
DEF_HELPER_3(fle_s, tl, env, tl, tl)
DEF_HELPER_3(flt_s, tl, env, tl, tl)
DEF_HELPER_3(feq_s, tl, env, tl, tl)
DEF_HELPER_3(fcvt_w_s, tl, env, tl, tl)
DEF_HELPER_3(fcvt_wu_s, tl, env, tl, tl)
DEF_HELPER_3(fcvt_l_s, tl, env, tl, tl)
DEF_HELPER_3(fcvt_lu_s, tl, env, tl, tl)
DEF_HELPER_3(fcvt_s_w, tl, env, tl, tl)
DEF_HELPER_3(fcvt_s_wu, tl, env, tl, tl)
DEF_HELPER_3(fcvt_s_l, tl, env, tl, tl)
DEF_HELPER_3(fcvt_s_lu, tl, env, tl, tl)
DEF_HELPER_2(fclass_s, tl, env, tl)
DEF_HELPER_5(fmadd_s, tl, env, tl, tl, tl, tl)
DEF_HELPER_5(fmadd_d, tl, env, tl, tl, tl, tl)
DEF_HELPER_5(fmsub_s, tl, env, tl, tl, tl, tl)
DEF_HELPER_5(fmsub_d, tl, env, tl, tl, tl, tl)
DEF_HELPER_5(fnmsub_s, tl, env, tl, tl, tl, tl)
DEF_HELPER_5(fnmsub_d, tl, env, tl, tl, tl, tl)
DEF_HELPER_5(fnmadd_s, tl, env, tl, tl, tl, tl)
DEF_HELPER_5(fnmadd_d, tl, env, tl, tl, tl, tl)

DEF_HELPER_4(fadd_d, tl, env, tl, tl, tl)



/* Special functions */
#ifndef CONFIG_USER_ONLY
DEF_HELPER_1(sret, tl, env)
DEF_HELPER_2(scall, tl, env, tl)
DEF_HELPER_1(read_count, tl, env)
DEF_HELPER_2(store_compare, void, env, tl)
DEF_HELPER_2(store_count, void, env, tl)
DEF_HELPER_1(tlb_flush, void, env)
#endif /* !CONFIG_USER_ONLY */
DEF_HELPER_1(wait, void, env)


#include "exec/def-helper.h"
