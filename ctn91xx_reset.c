#include "ctn91xx_reset.h"
#include "ctn91xx_util.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_event.h"
#include "ctn91xx_rpc.h"



#if HAS_MPEG_DMA
void mpeg_reset(ctn91xx_dev_t* dev)
{
    int i;
    //stop dma engine
    ctn91xx_write8(0x1, dev->mpeg_dma_base, MPEG_DMA_RESET);
    ctn91xx_write8(0x0, dev->mpeg_dma_base, MPEG_DMA_RESET);

    ctn91xx_write8(0x1, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_RESET);
    ctn91xx_write8(0x0, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_RESET);

    ctn91xx_busywait(dev, 1000);

#if !USE_LEON
    if(ctn91xx_read8(dev->mpeg_dma_base, MPEG_DMA_BUSY)) {
        ERROR("resetting while dma is busy, this is BAD");
    }
#endif


    //unmap the pages that were in use
    //initialize the current scatter-gather addresses for each mpeg device
    //lock those pages
    for(i=0; i<NUM_MPEG_DEVICES; i++) {
        int num_pages = default_num_pages_per_bank(i);
        vbuffer_t* vbuffer = &dev->mpeg_buffer[i];
        int last_index_to_unmap;
        int j;

        spin_lock_w_flags(&vbuffer->lock);

        last_index_to_unmap = WRAPPED_PAGE_INDEX(vbuffer, vbuffer->write_idx + (2*num_pages));

        for(j = vbuffer->notify_idx; j != last_index_to_unmap; j = (j+1) % vbuffer->npages) {
            if(vbuffer->lock_cnt[j] > 0) {
                ctn91xx_mpeg_unmap_page(dev, vbuffer, j, 0,i);
                vbuffer->lock_cnt[j] = 0;
            }
        }

        if(is_filter_stream(i)) {
            ctn91xx_write8(num_pages, dev->mpeg_filter_dma_base,
                           MPEG_FILTER_DMA_PAGES_PER_BANK_BASE + ((i+CTN91XX_ORIGIN_TUNER0-CTN91XX_ORIGIN_FILTER0)*4));
        } else {
            ctn91xx_write8(num_pages, dev->mpeg_dma_base, MPEG_DMA_PAGES_PER_BANK_BASE + (i*4));
        }
        ctn91xx_reset_dma_adjustments(dev, i);

        vbuffer->notify_idx = vbuffer->write_idx = vbuffer->read_idx = 0;

        vbuffer_map_bank(dev, vbuffer, num_pages, 0);

        vbuffer->write_idx = WRAPPED_PAGE_INDEX(vbuffer, vbuffer->write_idx + (num_pages));
        vbuffer_map_bank(dev, vbuffer, num_pages, 1);

        vbuffer->write_idx = vbuffer->read_idx;

        spin_unlock_w_flags(&vbuffer->lock);

        if(is_filter_stream(i)) {
            ctn91xx_write8(0, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_LOCK_BASE + ((i+CTN91XX_ORIGIN_TUNER0-CTN91XX_ORIGIN_FILTER0)*2));
            ctn91xx_write8(0, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_LOCK_BASE + ((i+CTN91XX_ORIGIN_TUNER0-CTN91XX_ORIGIN_FILTER0)*2));
        } else {
            ctn91xx_write8(0, dev->mpeg_dma_base, MPEG_DMA_LOCK_BASE + (i*2));
            ctn91xx_write8(0, dev->mpeg_dma_base, MPEG_DMA_LOCK_BASE + (i*2));
        }
    }

#if USE_MPEG_NOTIFY
    ctn91xx_write8(0x01, dev->mpeg_dma_base, MPEG_DMA_NOTIFY_ENABLE);
    //timeouts requires notify support
    ctn91xx_write32(dev->dma_timeout, dev->mpeg_dma_base, MPEG_DMA_TIMEOUT_VALUE);
    ctn91xx_write8(0x01, dev->mpeg_dma_base, MPEG_DMA_SET_TIMEOUT);
    ctn91xx_write8(0x01, dev->mpeg_dma_base, MPEG_DMA_TIMER_ENABLE);

    //timeouts requires notify support
    ctn91xx_write8(0x01, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_NOTIFY_ENABLE);
    ctn91xx_write32(dev->dma_timeout, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_TIMEOUT_VALUE);
    ctn91xx_write8(0x01, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_SET_TIMEOUT);
    ctn91xx_write8(0x01, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_TIMER_ENABLE);
#endif

    ctn91xx_write8(0x01, dev->mpeg_dma_base, MPEG_DMA_WRITE_ENABLE);
    ctn91xx_write8(0x01, dev->mpeg_dma_base, MPEG_DMA_INTERRUPT_ENABLE);

    ctn91xx_write8(0x01, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_WRITE_ENABLE);
    ctn91xx_write8(0x01, dev->mpeg_filter_dma_base, MPEG_FILTER_DMA_INTERRUPT_ENABLE);
}
#endif

void ctn91xx_reset_all(ctn91xx_dev_t* dev)
{
    WRITE_LOCK();
    INFO("reset all");


    ctn91xx_event_reset(dev);

#if HAS_MPEG_DMA
    mpeg_reset(dev);
#endif
    WRITE_UNLOCK();
}


