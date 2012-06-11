#ifndef CTN91XX_NET_H
#define CTN91XX_NET_H

#include "ctn91xx.h"
#define CTN91XX_NET_TX_TIMEOUT  (6*HZ)

int ctn91xx_net_init( ctn91xx_dev_t* dev );
int ctn91xx_net_deinit( ctn91xx_dev_t* dev );

irqreturn_t ctn91xx_net_isr(int irq, void *ptr);
void ctn91xx_net_wake_up( ctn91xx_dev_t* dev );


void ctn91xx_net_rx_skb( ctn91xx_dev_t* dev, struct sk_buff* skb, uint16_t rx_len );

#endif
