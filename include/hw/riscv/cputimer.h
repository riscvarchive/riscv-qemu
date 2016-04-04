uint64_t rtc_read(CPURISCVState *env);
uint64_t rtc_read_with_delta(CPURISCVState *env);
uint64_t instret_read(CPURISCVState *env);
static inline uint64_t instret_read_with_delta(CPURISCVState *env);
