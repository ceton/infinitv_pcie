#ifndef CTN91XX_MPEG_H
#define CTN91XX_MPEG_H

#include "ctn91xx.h"


#if HAS_MPEG_DMA

#define WRAPPED_PAGE_INDEX(vbuffer, idx) ((idx) % (vbuffer)->npages)
#define PAGE_INDEX_TO_PTR(vbuffer, idx) ((vbuffer)->buffers[(idx)])

int is_filter_stream(int stream_index);

int ctn91xx_init_mpeg(ctn91xx_dev_t* dev);
void ctn91xx_deinit_mpeg(ctn91xx_dev_t* dev);
void ctn91xx_reinit_mpeg_registers( ctn91xx_dev_t* dev );

uint8_t ctn91xx_interrupt_mpeg_notify(ctn91xx_dev_t* dev, uint8_t origin, uint8_t notify_cnt, uint16_t bytes_per_page);
void ctn91xx_interrupt_mpeg_dma(ctn91xx_dev_t* dev, uint8_t origin, int bank);

void ctn91xx_reset_dma_adjustments(ctn91xx_dev_t* dev, uint32_t stream_index);

int ctn91xx_ioctl_mpeg_read(ctn91xx_dev_t* dev, int minor, unsigned long arg);

void ctn91xx_mpeg_unmap_page(ctn91xx_dev_t* dev, vbuffer_t* vbuffer, int notify_index, uint16_t bytes_per_page, uint8_t origin);
void vbuffer_map_bank(ctn91xx_dev_t* dev, vbuffer_t* vbuffer, int num_pages, int bank);

int pages_per_mpeg_device(int stream_index);

int num_pages_per_bank(ctn91xx_dev_t* dev, int stream_index);
int num_pages_per_notify(ctn91xx_dev_t* dev, int stream_index);
int mpeg_bytes_per_page_from_stream(ctn91xx_dev_t* dev, int stream_index);

int default_num_pages_per_bank(int stream_index);
int default_num_pages_per_notify(int stream_index);
int default_mpeg_bytes_per_page_from_stream(int stream_index);

void mpeg_vbuffer_from_tuner(uint8_t tuner, ctn91xx_dev_t* dev,
                             vbuffer_t** vbuffer);

void mpeg_vbuffer_from_tuner_index(uint8_t tuner_index, ctn91xx_dev_t* dev,
                             vbuffer_t** vbuffer);

int mpeg_data_ready(vbuffer_t* vbuffer);

ssize_t ctn91xx_mpeg_kernel_read( vbuffer_t* vbuffer, char* data, size_t count );


//Debug

int ctn91xx_print_vbuffer_state(ctn91xx_dev_t* dev, unsigned long arg);
int ctn91xx_mpeg_dma_stats( ctn91xx_dev_t* dev, unsigned long arg );


#endif

#endif
