#include "ctn91xx_rpc.h"
#include "ctn91xx_util.h"
#include "ctn91xx_net.h"


int ctn91xx_msg_buffer_wait_not_busy( ctn91xx_dev_t* dev )
{
    int tries_left = 10;
    struct net_device* ndev = dev->net_dev;
    ctn91xx_net_priv_t* priv = netdev_priv(ndev);

    spin_lock_w_flags( &priv->lock );

recheck:
    if( !priv->tx_outstanding ) {
        priv->tx_outstanding = 1;
        spin_unlock_w_flags( &priv->lock );
        return 0;
    }

    spin_unlock_w_flags( &priv->lock );

    //wait longer than the net timeout
    if( !wait_event_timeout(
                dev->msg_buffer_wait_queue,
                priv->tx_outstanding == 0,
                CTN91XX_NET_TX_TIMEOUT + HZ ) ) {
        ERROR("rpc send timed out");
        return -EBUSY;
    } else {

        if( tries_left <= 0 ) {
            //we've met our condition 10 times, but still haven't been able to
            //send. Just give up now.
            return -EBUSY;
        }

        spin_relock_w_flags( &priv->lock );
        tries_left--;
        goto recheck;
    }
}


void ctn91xx_rpc_send_common( ctn91xx_dev_t* dev, rpc_send_t* rpc )
{
    uint32_t msg_buffer_buffer = MSG_BUFFER_BUFFER;

    ctn91xx_write32( MSG_BUFFER_TAG_CONTROL, dev->msg_base, MSG_BUFFER_LOCAL_TAG );
    ctn91xx_write32( MSG_BUFFER_SLOT_NUMBER(dev) | MSG_BUFFER_TAG_CONTROL, dev->msg_base, MSG_BUFFER_TAG );

    ctn91xx_write8( rpc->payload_start, dev->msg_base, msg_buffer_buffer );

    if( rpc->payload_start ) {

        ctn91xx_write32( htonl( rpc->section_length ), dev->msg_base, msg_buffer_buffer + 4 );
        ctn91xx_write16( rpc->length + 8, dev->msg_base, MSG_BUFFER_MSG_LEN );
        memcpy( dev->msg_base + msg_buffer_buffer + 8, dev->rpc_buffer, rpc->length );

    } else {
        ctn91xx_write16( rpc->length + 4, dev->msg_base, MSG_BUFFER_MSG_LEN );
        memcpy( dev->msg_base + msg_buffer_buffer + 4, dev->rpc_buffer, rpc->length );
    }
    ctn91xx_write8( 1, dev->msg_base, MSG_BUFFER_MSG_AVAIL );
}



int ctn91xx_rpc_send_kernel( ctn91xx_dev_t* dev, rpc_send_t* rpc )
{
    if( ctn91xx_msg_buffer_wait_not_busy( dev ) < 0 ) {
        return -EBUSY;
    }

    memcpy( dev->rpc_buffer, rpc->msg, rpc->length );
    ctn91xx_rpc_send_common( dev, rpc );
    return 0;
}

int ctn91xx_rpc_send( ctn91xx_dev_t* dev, unsigned long arg, int compat )
{
    if( ctn91xx_msg_buffer_wait_not_busy( dev ) < 0 ) {
        return -EBUSY;
    }

    if( compat ) {
        rpc_send_compat_t rpc32 = {};
        rpc_send_t rpc = {};

        if( copy_from_user( &rpc32, (__user rpc_send_compat_t*)arg, sizeof( rpc_send_compat_t ) ) ) {
            return -EINVAL;
        }

        if( copy_from_user( dev->rpc_buffer, (char __user*)(unsigned long)rpc32.msg, rpc32.length ) ) {
            return -EINVAL;
        }

        rpc.payload_start = rpc32.payload_start;
        rpc.length = rpc32.length;
        rpc.section_length = rpc32.section_length;

        ctn91xx_rpc_send_common( dev, &rpc );
        return 0;
    } else {
        rpc_send_t rpc = {};

        if( copy_from_user( &rpc, (__user rpc_send_t*)arg, sizeof( rpc_send_t ) ) ) {
            return -EINVAL;
        }

        if( copy_from_user( dev->rpc_buffer, rpc.msg, rpc.length ) ) {
            return -EINVAL;
        }

        ctn91xx_rpc_send_common( dev, &rpc );
        return 0;
    }
}

