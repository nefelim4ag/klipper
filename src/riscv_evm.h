#include "ctr.h" // DECL_CTR

// Declare a function export as a platform call
#define DECL_FW_CALL(RET, FUNC) \
    DECL_CTR("DECL_FW_CALL " __stringify(RET) "|" \
        __stringify(FUNC))

#define DECL_FW_CALL5(RET, FUNC, A0, A1, A2, A3, A4) \
    DECL_CTR("DECL_FW_CALL " __stringify(RET) "|" \
        __stringify(FUNC) "|" __stringify(A0) "|" __stringify(A1) "|" \
        __stringify(A2) "|" __stringify(A3) "|" __stringify(A4))

#define DECL_FW_CALL4(RET, FUNC, A0, A1, A2, A3) \
    DECL_CTR("DECL_FW_CALL " __stringify(RET) "|" \
        __stringify(FUNC) "|" __stringify(A0) "|" __stringify(A1) "|" \
        __stringify(A2) "|" __stringify(A3))

#define DECL_FW_CALL3(RET, FUNC, A0, A1, A2) \
    DECL_CTR("DECL_FW_CALL " __stringify(RET) "|" \
        __stringify(FUNC) "|" __stringify(A0) "|" __stringify(A1) "|" \
        __stringify(A2))

#define DECL_FW_CALL2(RET, FUNC, A0, A1) \
    DECL_CTR("DECL_FW_CALL " __stringify(RET) "|" \
        __stringify(FUNC) "|" __stringify(A0) "|" __stringify(A1))

#define DECL_FW_CALL1(RET, FUNC, A0) \
    DECL_CTR("DECL_FW_CALL " __stringify(RET) "|" \
        __stringify(FUNC) "|" __stringify(A0))

void evm_response(uint8_t oid, uint8_t rlen, uint8_t *rdata);