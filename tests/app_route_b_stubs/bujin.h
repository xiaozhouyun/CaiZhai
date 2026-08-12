#ifndef TEST_BUJIN_H
#define TEST_BUJIN_H

typedef struct {
    unsigned int unused;
} Emm_TxStatus_t;

void Emm_GetTxStatus(Emm_TxStatus_t *status);

#endif
