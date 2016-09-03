#include "ctn91xx_net.h"
#include "ctn91xx_kal.h"
#include "ctn91xx_util.h"


#define PRINT_TRAFFIC 0


static int ctn91xx_net_open( struct net_device *ndev );
static int ctn91xx_net_start_xmit( struct sk_buff *skb, struct net_device *ndev );
static int ctn91xx_net_stop( struct net_device *ndev );
static struct net_device_stats* ctn91xx_net_get_stats( struct net_device *ndev );
static void ctn91xx_net_tx_timeout( struct net_device* ndev );
static void ctn91xx_handle_tx( ctn91xx_dev_t* dev, ctn91xx_net_priv_t* priv );

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static int ctn91xx_net_set_mac_addr(struct net_device *netdev, void *p);

static struct net_device_ops ctn91xx_net_device_ops = {
    .ndo_open = ctn91xx_net_open,
    .ndo_start_xmit = ctn91xx_net_start_xmit,
    .ndo_stop = ctn91xx_net_stop,
    .ndo_get_stats = ctn91xx_net_get_stats,
    .ndo_set_mac_address = ctn91xx_net_set_mac_addr,
    .ndo_tx_timeout = ctn91xx_net_tx_timeout,
};
#endif


int ctn91xx_net_init( ctn91xx_dev_t* dev )
{
    struct net_device* netdev = NULL;
    ctn91xx_net_priv_t* priv = NULL;
    int i;

    netdev = alloc_etherdev( sizeof( ctn91xx_net_priv_t ) );

    if( netdev == NULL ) {
        ERROR("unable to alloc new ethernet");
        return -ENOMEM;
    }

#if USE_PCI
    SET_NETDEV_DEV( netdev, &dev->pdev->dev );
#endif

    priv = netdev_priv(netdev);
    priv->ctn91xx_dev = dev;
    spin_lock_init( &priv->lock );

    snprintf( netdev->name, 20, "ctn%d", dev->board_number );

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
    netdev->netdev_ops = &ctn91xx_net_device_ops;
#else
    netdev->open = ctn91xx_net_open;
    netdev->hard_start_xmit = ctn91xx_net_start_xmit;
    netdev->stop = ctn91xx_net_stop;
    netdev->get_stats = ctn91xx_net_get_stats;
    netdev->tx_timeout = ctn91xx_net_tx_timeout;
#endif
    netdev->watchdog_timeo = CTN91XX_NET_TX_TIMEOUT;

    i = register_netdev( netdev );

    if( i ) {
        ERROR("failed to register netdev");
        free_netdev( netdev );
        return -EINVAL;
    }

#if USE_PCI
    netdev->dev_addr[0] = 0x00;
    netdev->dev_addr[1] = 0x22;
    netdev->dev_addr[2] = 0x2c;
    netdev->dev_addr[3] = 0xff;
    netdev->dev_addr[4] = 0xff;
    netdev->dev_addr[5] = 0xff - dev->board_number;
#else
    random_ether_addr(netdev->dev_addr);
#endif


    netdev->irq = dev->irq;
    netdev->base_addr = (unsigned long)dev->hw_reg_base;

    dev->net_dev = netdev;

#if USE_PCI
    ctn91xx_write8( 1, dev->msg_base, MSG_BUFFER_INT_ENABLE );
#endif


    return 0;
}

int ctn91xx_net_deinit( ctn91xx_dev_t* dev )
{
    ctn91xx_write32( 0, dev->msg_base, MSG_BUFFER_TAG );
    if(dev->net_dev) {
        unregister_netdev( dev->net_dev );
        free_netdev( dev->net_dev );
    }
    return 0;
}

static int ctn91xx_net_open( struct net_device *ndev )
{
    netif_start_queue( ndev );
    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static int ctn91xx_net_set_mac_addr(struct net_device *netdev, void *p)
{
    struct sockaddr *addr=p;

    memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

    sdump_buffer(netdev->dev_addr, 6, "MAC Address");
    return 0;
}
#endif


static int ctn91xx_net_start_xmit( struct sk_buff *skb, struct net_device *ndev )
{
    ctn91xx_net_priv_t* priv = netdev_priv(ndev);
    ctn91xx_dev_t* dev = priv->ctn91xx_dev;
    int ret = 0;

    uint32_t msg_buffer_buffer = MSG_BUFFER_BUFFER;

    spin_lock_w_flags( &priv->lock );

    if( !priv->tx_outstanding ) {

        //uint32_t tag = ctn91xx_read32( dev->msg_base, MSG_BUFFER_TAG );
        //uint8_t valid = ( tag >> 16 ) & 0x1;
        //
        //TODO check ready bit in tag to see if other side is up...if not up,
        //stop the queue and set a timer to check it periodically, so we
        //can netif_start_queue
        //
        //NOTE: windows driver does not set valid bit currently

        priv->tx_outstanding = 1;
        priv->tx_skb = skb;

        spin_unlock_w_flags( &priv->lock );

        ctn91xx_write32( MSG_BUFFER_TAG_ETHERNET, dev->msg_base, MSG_BUFFER_LOCAL_TAG );
        ctn91xx_write32( MSG_BUFFER_SLOT_NUMBER(dev) | MSG_BUFFER_TAG_ETHERNET, dev->msg_base, MSG_BUFFER_TAG );

        ctn91xx_write16( skb->len, dev->msg_base, MSG_BUFFER_MSG_LEN );
        memcpy( dev->msg_base + msg_buffer_buffer, skb->data, skb->len );

#if PRINT_TRAFFIC
        sdump_buffer( skb->data, skb->len, "tx");
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,7,0)
        netif_trans_update(ndev);
#else
        ndev->trans_start = jiffies;
#endif
        ctn91xx_write8( 1, dev->msg_base, MSG_BUFFER_MSG_AVAIL );

    } else {
        netif_stop_queue( ndev );
        ret = NETDEV_TX_BUSY;
        spin_unlock_w_flags( &priv->lock );
    }

    return ret;
}

static void ctn91xx_net_tx_timeout( struct net_device* net_dev )
{
    ctn91xx_net_priv_t* priv = netdev_priv(net_dev);
    ctn91xx_dev_t* dev = priv->ctn91xx_dev;
    uint32_t tag = ctn91xx_read32( dev->msg_base, MSG_BUFFER_TAG );
    uint8_t valid = ( tag >> 16 ) & 0x1;

    spin_lock_w_flags( &priv->lock );

    INFO("tx timeout (eth_pend=%d, tx_pen=%d recv=%d skb=%d valid=%d)",
         (ctn91xx_read32( dev->msg_base, MSG_BUFFER_LOCAL_TAG ) & 0xff ) == MSG_BUFFER_TAG_ETHERNET,
         priv->tx_outstanding,
         ctn91xx_read8( dev->msg_base, MSG_BUFFER_MSG_RECV ),
         priv->tx_skb != NULL,
         valid);
    ctn91xx_handle_tx( dev, priv );

    spin_unlock_w_flags( &priv->lock );
}

static int ctn91xx_net_stop( struct net_device *ndev )
{
    netif_stop_queue (ndev);
    return 0;
}

static struct net_device_stats* ctn91xx_net_get_stats( struct net_device *ndev )
{
    ctn91xx_net_priv_t* priv = netdev_priv(ndev);
    return &priv->stats;
}

//lock must be held when calling
static void ctn91xx_handle_tx( ctn91xx_dev_t* dev, ctn91xx_net_priv_t* priv )
{
    if( !priv->tx_outstanding ) {
        ERROR("unexpected tx oustanding == 0");
    }

    if( (ctn91xx_read32( dev->msg_base, MSG_BUFFER_LOCAL_TAG ) & 0xff ) == MSG_BUFFER_TAG_ETHERNET) {
        priv->tx_outstanding = 0;
        ctn91xx_write8( 0, dev->msg_base, MSG_BUFFER_INT_ACK_RECV );
    }

    if( priv->tx_skb ) {
        priv->stats.tx_bytes += priv->tx_skb->len;
        priv->stats.tx_packets++;
        dev_kfree_skb_irq( priv->tx_skb );
        priv->tx_skb = NULL;
    }

    ctn91xx_net_wake_up( dev );
    wake_up( &dev->msg_buffer_wait_queue );
}

void ctn91xx_net_rx_skb( ctn91xx_dev_t* dev, struct sk_buff* skb, uint16_t rx_len )
{
    struct net_device* netdev = dev->net_dev;
    ctn91xx_net_priv_t* priv = netdev_priv(netdev);

    spin_lock_w_flags( &priv->lock );

    skb->dev = netdev;
    skb_put( skb, rx_len );
    skb->protocol = eth_type_trans( skb, netdev );
    netif_rx( skb );

    netdev->last_rx = jiffies;
    priv->stats.rx_bytes += rx_len;
    priv->stats.rx_packets++;

    spin_unlock_w_flags( &priv->lock );
}

irqreturn_t ctn91xx_net_isr(int irq, void *ptr)
{
    ctn91xx_dev_t* dev = ptr;
    struct net_device* netdev = dev->net_dev;
    ctn91xx_net_priv_t* priv = netdev_priv(netdev);
    irqreturn_t ret = IRQ_NONE;
    struct sk_buff* skb;
    uint16_t rx_len;
    uint32_t msg_buffer_buffer = MSG_BUFFER_BUFFER;

    spin_lock_w_flags( &priv->lock );

    if( ctn91xx_read8( dev->msg_base, MSG_BUFFER_MSG_AVAIL ) &&
            ( ctn91xx_read32( dev->msg_base, MSG_BUFFER_TAG ) & 0xff ) == MSG_BUFFER_TAG_ETHERNET ) {

        ret = IRQ_HANDLED;
        dev->stats.isr_net_rx_count++;

        rx_len = ctn91xx_read16( dev->msg_base, MSG_BUFFER_MSG_LEN );

        ctn91xx_write8( 0, dev->msg_base, MSG_BUFFER_INT_ACK_AVAIL );

        if( rx_len > 0 ) {
            skb = dev_alloc_skb( rx_len );

            if( skb ) {
                skb->dev = netdev;

                memcpy( skb->data, dev->msg_base + msg_buffer_buffer, rx_len );

#if PRINT_TRAFFIC
                sdump_buffer( skb->data, rx_len, "rx");
#endif
                skb_put( skb, rx_len );

                skb->protocol = eth_type_trans( skb, netdev );
                netif_rx( skb );

                netdev->last_rx = jiffies;
                priv->stats.rx_bytes += rx_len;
                priv->stats.rx_packets++;
            } else {
                if( printk_ratelimit() ) {
                    INFO("Memory squeeze, dropping packet");
                }
                priv->stats.rx_dropped++;
            }
        } else {
            dev->stats.isr_net_zero_count++;
        }

        ctn91xx_write8( 1, dev->msg_base, MSG_BUFFER_MSG_RECV );
    }

    if( ctn91xx_read8( dev->msg_base, MSG_BUFFER_MSG_RECV ) &&
            ctn91xx_read32( dev->msg_base, MSG_BUFFER_LOCAL_TAG ) == MSG_BUFFER_TAG_ETHERNET ) {

        dev->stats.isr_net_tx_count++;
        ret = IRQ_HANDLED;

        ctn91xx_handle_tx( dev, priv );
    }

    spin_unlock_w_flags( &priv->lock );
    return ret;
}

void ctn91xx_net_wake_up( ctn91xx_dev_t* ctn91xx_dev )
{
    if( netif_queue_stopped( ctn91xx_dev->net_dev ) ) {
        netif_wake_queue( ctn91xx_dev->net_dev );
    }
}

