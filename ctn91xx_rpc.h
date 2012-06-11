#ifndef CTN91XX_RPC_H
#define CTN91XX_RPC_H

#include "ctn91xx.h"

int ctn91xx_rpc_send( ctn91xx_dev_t* dev, unsigned long arg, int compat );
int ctn91xx_rpc_send_kernel( ctn91xx_dev_t* dev, rpc_send_t* rpc );

#endif
