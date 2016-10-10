/* Exceptions */
DEF_HELPER_2(raise_exception, noreturn, env, i32)
DEF_HELPER_1(raise_exception_debug, noreturn, env)
DEF_HELPER_3(raise_exception_mbadaddr, noreturn, env, i32, tl)

/* Floating Point - fused */
DEF_HELPER_FLAGS_5(fmadd_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)
DEF_HELPER_FLAGS_5(fmadd_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)
DEF_HELPER_FLAGS_5(fmsub_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)
DEF_HELPER_FLAGS_5(fmsub_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)
DEF_HELPER_FLAGS_5(fnmsub_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)
DEF_HELPER_FLAGS_5(fnmsub_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)
DEF_HELPER_FLAGS_5(fnmadd_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)
DEF_HELPER_FLAGS_5(fnmadd_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64, i64)

/* Floating Point - Single Precision */
DEF_HELPER_FLAGS_4(fadd_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fsub_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fmul_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fdiv_s, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_3(fmin_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fsqrt_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fle_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(flt_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(feq_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_w_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_wu_s, TCG_CALL_NO_RWG, tl, env, i64, i64)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_3(fcvt_l_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_lu_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
#endif
DEF_HELPER_FLAGS_3(fcvt_s_w, TCG_CALL_NO_RWG, i64, env, tl, i64)
DEF_HELPER_FLAGS_3(fcvt_s_wu, TCG_CALL_NO_RWG, i64, env, tl, i64)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_3(fcvt_s_l, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_s_lu, TCG_CALL_NO_RWG, i64, env, i64, i64)
#endif
DEF_HELPER_FLAGS_2(fclass_s, TCG_CALL_NO_RWG, tl, env, i64)

/* Floating Point - Double Precision */
DEF_HELPER_FLAGS_4(fadd_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fsub_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fmul_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_4(fdiv_d, TCG_CALL_NO_RWG, i64, env, i64, i64, i64)
DEF_HELPER_FLAGS_3(fmin_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fmax_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_s_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_d_s, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fsqrt_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fle_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(flt_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(feq_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_w_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_wu_d, TCG_CALL_NO_RWG, tl, env, i64, i64)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_3(fcvt_l_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_lu_d, TCG_CALL_NO_RWG, i64, env, i64, i64)
#endif
DEF_HELPER_FLAGS_3(fcvt_d_w, TCG_CALL_NO_RWG, i64, env, tl, i64)
DEF_HELPER_FLAGS_3(fcvt_d_wu, TCG_CALL_NO_RWG, i64, env, tl, i64)
#if defined(TARGET_RISCV64)
DEF_HELPER_FLAGS_3(fcvt_d_l, TCG_CALL_NO_RWG, i64, env, i64, i64)
DEF_HELPER_FLAGS_3(fcvt_d_lu, TCG_CALL_NO_RWG, i64, env, i64, i64)
#endif
DEF_HELPER_FLAGS_2(fclass_d, TCG_CALL_NO_RWG, tl, env, i64)

/* Special functions */
DEF_HELPER_3(csrrw, tl, env, tl, tl)
DEF_HELPER_4(csrrs, tl, env, tl, tl, tl)
DEF_HELPER_4(csrrc, tl, env, tl, tl, tl)
#ifndef CONFIG_USER_ONLY
DEF_HELPER_2(sret, tl, env, tl)
DEF_HELPER_2(mret, tl, env, tl)
DEF_HELPER_1(tlb_flush, void, env)
DEF_HELPER_1(fence_i, void, env)
#endif
