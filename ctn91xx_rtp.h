#ifndef CTN91XX_RTP_H
#define CTN91XX_RTP_H

#include "ctn91xx.h"

#define MP2T_PAYLOAD_TYPE 33
#define MP2T_TIMESTAMP_CLOCK 90000

#define SIZEOF_CONTROL_MSG_FIRST_HEADER 5
#define SIZEOF_CONTROL_MSG_HEADER       1
#define SIZEOF_CONTROL_MSG_RTP_SETUP    28
#define SIZEOF_CONTROL_MSG_RTP_DESTROY  4
#define SIZEOF_CONTROL_MSG_PBDA_TAG_TABLE (4+188)
#define NUM_MPEG_STREAMS 6

#define MPEG_TS_PKT_SIZE 188
#define RTP_HDR_SIZE 12

#define RTP_NOTIFY_COUNT 8
#define RTP_NOTIFY_PER_PKT 2

#define BUFFER_TS_COUNT 21
#define BRIDGED_TS_COUNT 7
#define NUM_TS_PKTS_PER_RTP_PKT BRIDGED_TS_COUNT
#define RTP_SINK_BUFFER_SIZE ((NUM_TS_PKTS_PER_RTP_PKT*MPEG_TS_PKT_SIZE)+RTP_HDR_SIZE)

void ctn91xx_rtp_setup(ctn91xx_dev_t* dev, uint8_t* buf, uint32_t len);
void ctn91xx_rtp_destroy(ctn91xx_dev_t* dev, uint8_t* buf, uint32_t len);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
void ctn91xx_rtp_reader(void* work);
#else
void ctn91xx_rtp_reader(struct work_struct* work);
#endif

#endif
