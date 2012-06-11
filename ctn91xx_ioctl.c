#include "ctn91xx.h"
#include "ctn91xx_ioctl.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_reset.h"
#include "ctn91xx_rpc.h"
#include "ctn91xx_event.h"

static int
ctn91xx_get_stats( ctn91xx_dev_t* dev, unsigned long arg )
{
    if( copy_to_user( (void* __user)arg, &dev->stats, sizeof(ctn91xx_stats_t) ) ) {
        return -EINVAL;
    }

    return 0;
}

int ctn91xx_ioctl_handle(uint32_t cmd, unsigned long arg, ctn91xx_dev_t* dev, int compat)
{
    int ret = 0;
    switch(cmd) {
        case CTN91XX_IOCTL_RESET_ALL:
            ctn91xx_reset_all(dev);
            break;
#if HAS_MPEG_DMA
        case CTN91XX_IOCTL_VBUFFER_STATUS:
            ret = ctn91xx_print_vbuffer_state(dev, arg);
            break;
        case CTN91XX_IOCTL_DMA_STATS:
            ret = ctn91xx_mpeg_dma_stats( dev, arg );
            break;
#endif
        case CTN91XX_IOCTL_EVENT_READ:
            ret = ctn91xx_ioctl_event_read(dev, arg, compat);
            break;
        case CTN91XX_IOCTL_RPC_SEND:
            ret = ctn91xx_rpc_send( dev, arg, compat );
            break;
        case CTN91XX_IOCTL_GET_STATS:
            ret = ctn91xx_get_stats( dev, arg );
            break;
        case CTN91XX_IOCTL_IS_BOARD_BOOTED:
            ret = dev->board_booting;
            break;
        case TCGETS:
            //fopen calls this on us...it's the "is_a_tty" call
            //just tell it to go away
            ret = -EINVAL;
            break;
        default:
            ret = -EINVAL;
            WARNING("Unknown ioctl on ctn91xx device (%d).", cmd);
            break;
    }
    return ret;
}
