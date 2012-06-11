#include "ctn91xx_interrupt.h"
#include "ctn91xx_util.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_net.h"
#include "ctn91xx_event.h"
#include "ctn91xx_rtp.h"


static int ctn91xx_interrupt_is_mpeg_dma(ctn91xx_dev_t* dev)
{
    if(dev->int_enable) {
        uint8_t status = ctn91xx_read8(dev->mpeg_dma_base, MPEG_DMA_BANK_INTERRUPT_MASKED);
        return status == 1;
    } else {
        return 0;
    }
}

#if USE_LEON
static int ctn91xx_interrupt_is_mpeg_filter_dma(ctn91xx_dev_t* dev)
{
    if(dev->int_enable) {
        uint8_t status = ctn91xx_read8(dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_BANK_INTERRUPT_MASKED);
        return status == 1;
    } else {
        return 0;
    }
}
#endif


#define CONTROL_MSG_RTP_SETUP 0x1
#define CONTROL_MSG_RTP_DESTROY 0x2
#define CONTROL_MSG_MASK_TABLE_SETUP 0x3
#define CONTROL_MSG_SLOT_NUMBER 0x8

int ctn91xx_interrupt_rpc_recvd(ctn91xx_dev_t* dev, uint16_t msg_len, uint8_t origin)
{
    ctn91xx_event_t* event;

    uint32_t msg_buffer_buffer = MSG_BUFFER_BUFFER;
    uint8_t* payload = ((uint8_t*)dev->msg_base) + msg_buffer_buffer + 8 + 4;
    uint8_t payload_start = ctn91xx_read8( dev->msg_base, msg_buffer_buffer );
    uint8_t msg_type = ctn91xx_read8( dev->msg_base, msg_buffer_buffer + 8 );

#if USE_PCI
    if( payload_start ) {
        if( msg_type == CONTROL_MSG_MASK_TABLE_SETUP ) {
#if HAS_MPEG_DMA
            ctn91xx_reinit_mpeg_registers( dev );
#endif
            schedule_work(&dev->delayed_slot_number_worker);
            INFO("handled mask table setup");
            return 0;
        } else if( msg_type == CONTROL_MSG_RTP_SETUP ) {
            ctn91xx_rtp_setup(dev, payload, msg_len - 12);
            return 0;
        } else if( msg_type == CONTROL_MSG_RTP_DESTROY ) {
            ctn91xx_rtp_destroy(dev, payload, msg_len - 12);
            return 0;
        }
    }
#endif

    //event to user
    event = ctn91xx_event_new_full(dev,
                                   CTN91XX_EVENT_RPC_RECVD,
                                   msg_len,
                                   origin,
                                   0,//unused
                                   0//unused
                                  );

    if(!event) {
        INFO("dropping interrupt type %02x, no listeners", CTN91XX_EVENT_RPC_RECVD);
        return 0;
    }

    INFO("queueing rpc recvd: payload_start: %d msg_type: %d", payload_start, msg_type);

    ctn91xx_queue_event(dev, event);
    return 1;
}

int ctn91xx_interrupt_rpc_ack_recvd( ctn91xx_dev_t* dev )
{
    ctn91xx_event_t* event;

    //event to user
    event = ctn91xx_event_new_full(dev,
                                   CTN91XX_EVENT_RPC_ACK_RECVD,
                                   0,
                                   CTN91XX_ORIGIN_MSG_BUFFER,
                                   0,//unused
                                   0//unused
                                  );

    if(!event) {
        INFO("dropping interrupt type %02x, no listeners", CTN91XX_EVENT_RPC_ACK_RECVD);
        return 0;
    }

    ctn91xx_queue_event(dev, event);
    return 1;
}

irqreturn_t rpc_isr(int irq, void *ptr)
{
    ctn91xx_dev_t* dev = ptr;
    irqreturn_t ret_val = IRQ_NONE;
    uint16_t msg_len;
    struct net_device* ndev = dev->net_dev;
    ctn91xx_net_priv_t* priv = netdev_priv(ndev);

    spin_lock_w_flags( &priv->lock );

    if( ctn91xx_read8( dev->msg_base, MSG_BUFFER_MSG_AVAIL ) &&
            ( ctn91xx_read32( dev->msg_base, MSG_BUFFER_TAG ) & 0xff ) == MSG_BUFFER_TAG_CONTROL ) {

        ret_val = IRQ_HANDLED;

        ctn91xx_write8( 0, dev->msg_base, MSG_BUFFER_INT_ACK_AVAIL );

        msg_len = ctn91xx_read16( dev->msg_base, MSG_BUFFER_MSG_LEN );

        if( !ctn91xx_interrupt_rpc_recvd( dev, msg_len, CTN91XX_ORIGIN_MSG_BUFFER ) ) {
            ctn91xx_write8( 1, dev->msg_base, MSG_BUFFER_MSG_RECV );
        }
    }

    if( ctn91xx_read8( dev->msg_base, MSG_BUFFER_MSG_RECV ) &&
            ( ctn91xx_read32( dev->msg_base, MSG_BUFFER_LOCAL_TAG ) & 0xff ) == MSG_BUFFER_TAG_CONTROL ) {

        ret_val = IRQ_HANDLED;

        ctn91xx_write8( 0, dev->msg_base, MSG_BUFFER_INT_ACK_RECV );

        if( !priv->tx_outstanding ) {
            ERROR("unexpected rpc tx interrupt");
        } else {
            ctn91xx_interrupt_rpc_ack_recvd( dev );
            priv->tx_outstanding = 0;
        }

        ctn91xx_net_wake_up( dev );
        wake_up( &dev->msg_buffer_wait_queue );
    }

    spin_unlock_w_flags( &priv->lock );

    return ret_val;
}

#if USE_LEON
#endif

#if HAS_MPEG_DMA

irqreturn_t ctn91xx_mpeg_isr(int irq, void *ptr)
{
    ctn91xx_dev_t *dev = ptr;
    irqreturn_t ret_val = IRQ_NONE;
    int i;
    uint16_t notify_vector = 0;
    uint16_t notify_mask = 0;

    notify_vector = ctn91xx_read16(dev->mpeg_dma_base, MPEG_DMA_NOTIFY_VECTOR);

    if(notify_vector) {
        for(i=0; i<NUM_MPEG_DEVICES/2; i++) {

            notify_mask = (1 << i);
            if(notify_vector & notify_mask) {

                uint8_t origin = i + CTN91XX_ORIGIN_TUNER0;
                uint8_t notify_cnt = ctn91xx_read8(dev->mpeg_dma_base, MPEG_DMA_NOTIFY_BASE + (i*8));
                uint16_t bytes_per_page = ctn91xx_read16(dev->mpeg_dma_base, MPEG_DMA_NOTIFY_BYTES_PER_PAGE_BASE + (i*8));

                uint8_t pages_cleared = ctn91xx_interrupt_mpeg_notify(dev, origin, notify_cnt, bytes_per_page);

                if( pages_cleared != notify_cnt ) {
                    dev->stats.mismatched_notify_clear[i]++;
                }

                ctn91xx_write8(pages_cleared, dev->mpeg_dma_base, MPEG_DMA_NOTIFY_BASE + (i*8));
                ctn91xx_write8(0x1, dev->mpeg_dma_base, MPEG_DMA_NOTIFY_BASE + (i*8) + 1);
            }
        }
        ret_val = IRQ_HANDLED;
        ctn91xx_write32(dev->dma_timeout, dev->mpeg_dma_base, MPEG_DMA_TIMEOUT_VALUE);
        ctn91xx_write8(0x01, dev->mpeg_dma_base, MPEG_DMA_SET_TIMEOUT);
    }

#if USE_LEON
    notify_vector = ctn91xx_read16(dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_NOTIFY_VECTOR);

    if(notify_vector) {
        for(i=0; i<NUM_MPEG_DEVICES/2; i++) {
            notify_mask = (1 << i);
            if(notify_vector & notify_mask) {

                uint8_t origin = i + CTN91XX_ORIGIN_FILTER0;
                uint8_t notify_cnt;
                uint16_t bytes_per_page;
                uint8_t pages_cleared;

                notify_cnt = ctn91xx_read8(dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_NOTIFY_BASE + (i*8));
                bytes_per_page = ctn91xx_read16(dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_NOTIFY_BYTES_PER_PAGE_BASE + (i*8));

                if( notify_cnt > num_pages_per_bank( dev, origin - CTN91XX_ORIGIN_TUNER0 ) ) {
                    if( printk_ratelimit() ) {
                        ERROR("notify_cnt %d > bank size", notify_cnt );
                    }
                }

                pages_cleared = ctn91xx_interrupt_mpeg_notify(dev, origin, notify_cnt, bytes_per_page);

                ctn91xx_write8(pages_cleared, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_NOTIFY_BASE + (i*8));
                ctn91xx_write8(0x1, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_NOTIFY_BASE + (i*8) + 1);
            }
        }
        ret_val = IRQ_HANDLED;
        ctn91xx_write32(dev->dma_timeout, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_TIMEOUT_VALUE);
        ctn91xx_write8(0x01, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_SET_TIMEOUT);
    }
#endif

    if(ctn91xx_interrupt_is_mpeg_dma(dev)) {
        for(i=0; i<NUM_MPEG_DEVICES/2; i++) {
            uint8_t origin = i + CTN91XX_ORIGIN_TUNER0;
            uint8_t bank_lock = ctn91xx_read8(dev->mpeg_dma_base, MPEG_DMA_LOCK_BASE + (i*2));

            if(bank_lock) {

                uint8_t which_bank = ctn91xx_read8(dev->mpeg_dma_base, MPEG_DMA_LOCK_BASE + (i*2) + 1);
                ctn91xx_interrupt_mpeg_dma(dev, origin, which_bank);

                ctn91xx_write8(0, dev->mpeg_dma_base, MPEG_DMA_LOCK_BASE + (i*2));
            }
        }
        ret_val = IRQ_HANDLED;
    }

#if USE_LEON
    if(ctn91xx_interrupt_is_mpeg_filter_dma(dev)) {
        for(i=0; i<NUM_MPEG_DEVICES/2; i++) {
            uint8_t origin;
            uint8_t bank_lock;

            origin = i + CTN91XX_ORIGIN_FILTER0;
            bank_lock = ctn91xx_read8(dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_LOCK_BASE + (i*2));

            if(bank_lock) {
                uint8_t which_bank = ctn91xx_read8(dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_LOCK_BASE + (i*2) + 1);
                ctn91xx_interrupt_mpeg_dma(dev, origin, which_bank);
                ctn91xx_write8(0, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_LOCK_BASE + (i*2));
            }
        }
        ret_val = IRQ_HANDLED;
    }
#endif

    return ret_val;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
irqreturn_t ctn91xx_isr(int irq, void *ptr, struct pt_regs* regs)
#else
irqreturn_t ctn91xx_isr(int irq, void *ptr)
#endif
{
    ctn91xx_dev_t *dev = ptr;
    spin_lock_w_flags( &dev->isr_lock );

    dev->stats.isr_count++;

    if( !dev->int_enable ) {
        goto not_handled;
    }

    if( rpc_isr( irq, ptr ) == IRQ_HANDLED ) {
        dev->stats.isr_rpc_count++;
        goto handled;
    }

    if( ctn91xx_net_isr( irq, ptr ) == IRQ_HANDLED ) {
        dev->stats.isr_net_count++;
        goto handled;
    }


#if HAS_MPEG_DMA
    if( ctn91xx_mpeg_isr(irq,ptr) == IRQ_HANDLED ) {
        dev->stats.isr_mpeg_count++;
        goto handled;
    }
#endif


#if USE_LEON
#endif

not_handled:
    dev->stats.isr_not_handled++;
    spin_unlock_w_flags( &dev->isr_lock );
    return IRQ_NONE;
handled:
    spin_unlock_w_flags( &dev->isr_lock );
    return IRQ_HANDLED;
}


irqreturn_t ctn91xx_timer_isr(int irq, void* ptr)
{
    irqreturn_t ret = IRQ_NONE;
    ctn91xx_dev_t* dev = (ctn91xx_dev_t*)ptr;
    int i=0;

    for(i=0; i<NUM_TIMERS; i++) {

        uint8_t irqset = ctn91xx_read8(dev->timer_reg_base, TIMER_IRQ_REG(i));
        if(irqset) {
            INFO("timer interrupt for timer %d, clearing", i);
            ctn91xx_write8(0, dev->timer_reg_base, TIMER_IRQ_REG(i));
            ret = IRQ_HANDLED;
        }
    }

    return ret;
}



#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#define ktime_get ktime_get_real
#endif





