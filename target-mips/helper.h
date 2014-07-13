#include "exec/def-helper.h"

DEF_HELPER_3(raise_exception_err, noreturn, env, i32, int)
DEF_HELPER_2(raise_exception, noreturn, env, i32)

DEF_HELPER_3(mulhsu, tl, env, tl, tl)
//DEF_HELPER_2(riscv_exception, void, env, int)

//DEF_HELPER_1(read_count, tl, env)

DEF_HELPER_3(csrrw, tl, env, tl, tl)
DEF_HELPER_3(csrrs, tl, env, tl, tl)
DEF_HELPER_3(csrrc, tl, env, tl, tl)


/* Special functions */
#ifndef CONFIG_USER_ONLY
DEF_HELPER_1(sret, tl, env)
DEF_HELPER_2(scall, tl, env, tl)
DEF_HELPER_1(read_count, tl, env)
DEF_HELPER_2(store_compare, void, env, tl)
DEF_HELPER_2(store_count, void, env, tl)
DEF_HELPER_1(tlb_flush, void, env)
DEF_HELPER_1(tlbwi, void, env)
DEF_HELPER_1(tlbwr, void, env)
DEF_HELPER_1(tlbp, void, env)
DEF_HELPER_1(tlbr, void, env)
#endif /* !CONFIG_USER_ONLY */
DEF_HELPER_1(wait, void, env)


#include "exec/def-helper.h"
