#include "ctn91xx.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_rtp.h"
#include "ctn91xx_net.h"

void ctn91xx_rtp_setup(ctn91xx_dev_t* dev, uint8_t* buf, uint32_t len)
{
    vbuffer_t* vbuffer = NULL;
    rtp_state_t* rtp = NULL;
    uint32_t tuner_index;

    if( len < SIZEOF_CONTROL_MSG_RTP_SETUP ) {
        WARNING("len %d too small", len);
        return;
    }

    tuner_index = ntohl( *((uint32_t*)&buf[0] ) );

    if( tuner_index >= NUM_MPEG_STREAMS ) {
        WARNING("tuner index %08x was too large", tuner_index);
        return;
    }

#if HAS_MPEG_DMA
    mpeg_vbuffer_from_tuner_index(tuner_index, dev, &vbuffer);
#endif

    if( !vbuffer ) {
        return;
    }

    rtp = &vbuffer->rtp_state;
    buf += 4;

    memcpy( rtp->remote_hw_addr, buf, 6 );
    buf += 6;

    memcpy( rtp->local_hw_addr, buf, 6 );
    buf += 6;

    rtp->remote_ip_addr = *((uint32_t*)&buf[0]);
    buf += 4;

    rtp->local_ip_addr = *((uint32_t*)&buf[0]);
    buf += 4;

    rtp->remote_port = *((uint16_t*)&buf[0]);
    buf += 2;

    rtp->local_port = *((uint16_t*)&buf[0]);
    buf += 2;

    if( !rtp->got_setup ) {

        rtp_header_t* hdr = (rtp_header_t*)rtp->buffer;

        rtp->buffer_used = RTP_HDR_SIZE;

        hdr->version = 2;
        hdr->padding = 0;
        hdr->extension = 0;
        hdr->csrc_len = 0;
        hdr->marker = 0;
        hdr->payload = MP2T_PAYLOAD_TYPE;

        rtp->got_setup = 1;
        rtp->running = 1;

        dev->mpeg_user_cnt[tuner_index]++;

        vbuffer->read_idx = vbuffer->notify_idx;
    }
}

void ctn91xx_rtp_destroy(ctn91xx_dev_t* dev, uint8_t* buf, uint32_t len)
{
    vbuffer_t* vbuffer = NULL;
    rtp_state_t* rtp = NULL;
    uint32_t tuner_index;

    if( len < SIZEOF_CONTROL_MSG_RTP_DESTROY ) {
        WARNING("len %d too small", len);
        return;
    }

    tuner_index = ntohl( *((uint32_t*)&buf[0] ) );

    if( tuner_index >= NUM_MPEG_STREAMS ) {
        WARNING("tuner index %d was too large", tuner_index);
        return;
    }

#if HAS_MPEG_DMA
    mpeg_vbuffer_from_tuner_index(tuner_index, dev, &vbuffer);
#endif

    if( !vbuffer ) {
        return;
    }

    rtp = &vbuffer->rtp_state;

    memset( rtp->remote_hw_addr, 0, 6 );
    memset( rtp->local_hw_addr, 0, 6 );
    rtp->remote_ip_addr = 0;
    rtp->local_ip_addr = 0;
    rtp->remote_port = 0;
    rtp->local_port = 0;

    rtp->got_setup = 0;
    rtp->running = 0;

    if( dev->mpeg_user_cnt[tuner_index] > 0 ) {
        dev->mpeg_user_cnt[tuner_index]--;
    }
}

uint16_t ip_hdr_chksum( uint8_t* ip, int len )
{
    uint16_t* buf = (uint16_t*)ip;
    int sum = 0;

    while( len > 1 ) {
        sum += *buf;
        buf++;
        if( sum & 0x80000000 ) {
            /* if high order bit set, fold */
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
        len -= 2;
    }

    if( len ) {
        /* take care of left over byte */
        sum += (unsigned short) *(unsigned char *)buf;
    }

    while( sum >> 16 ) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return (uint16_t)~sum;
}


#define ETH_PACKET_SIZE(rtp) (rtp->buffer_used + 14 /* eth */ + 20 /* ip */ + 8 /* udp */)
#define IP_PACKET_SIZE(rtp)  (rtp->buffer_used + 20 /* ip */ + 8 /* udp */)
#define UDP_PACKET_SIZE(rtp) (rtp->buffer_used + 8 /* udp */)
#define RTP_PACKET_SIZE(rtp) (rtp->buffer_used)


static void rtp_send_packet( vbuffer_t* vbuffer )
{
    ctn91xx_dev_t* dev = vbuffer->dev;
    rtp_state_t* rtp = &vbuffer->rtp_state;
    struct sk_buff* skb;
    rtp_header_t* hdr = (rtp_header_t*)rtp->buffer;
    uint8_t* buf = NULL;
    uint16_t rx_len = ETH_PACKET_SIZE(rtp);
    uint16_t ip_chk_sum = 0;

    hdr->seq_no = htons( rtp->seq_no++ );
    hdr->timestamp = MP2T_TIMESTAMP_CLOCK;//TODO get actual timstamp
    hdr->ssrc = htonl( rtp->ssrc );

    skb = dev_alloc_skb( rx_len );

    if( !skb ) {
        if( printk_ratelimit() ) {
            WARNING("memory squeeze, dropping packet");
        }
        return;
    }

    buf = skb->data;

    memcpy( buf, rtp->remote_hw_addr, 6 );
    memcpy( buf + 6, rtp->local_hw_addr, 6 );
    //ether type = IP
    buf[12] = 0x08;
    buf[13] = 0x00;
    buf += 14;

    //ipv4 header
    buf[0] = 0x45;
    buf[1] = 0x00;

    //ip length
    *((uint16_t*)&buf[2]) = htons( IP_PACKET_SIZE( rtp ) );

    //identification
    *((uint16_t*)&buf[4]) = htons( rtp->ip_seq_no++ );

    /* flags, fragment offset */
    buf[6] = 0;
    buf[7] = 0;
    /* ttl */
    buf[8] = 10;
    /* protocol = UDP */
    buf[9] = 0x11;

    /* checksum */
    *((uint16_t*)&buf[10]) = 0x0000;

    /* addrs */
    *((uint32_t*)&buf[12]) = rtp->local_ip_addr;
    *((uint32_t*)&buf[16]) = rtp->remote_ip_addr;

    /* go back and calc checksum */
    ip_chk_sum = ip_hdr_chksum( buf, 20 );
    *((uint16_t*)&buf[10]) = ip_chk_sum;

    buf += 20;

    //udp header
    *((uint16_t*)&buf[0]) = rtp->local_port;
    *((uint16_t*)&buf[2]) = rtp->remote_port;
    *((uint16_t*)&buf[4]) = htons( UDP_PACKET_SIZE( rtp ) );
    //checksum
    *((uint16_t*)&buf[6]) = 0x0000;
    buf += 8;

    memcpy( buf, rtp->buffer, RTP_PACKET_SIZE( rtp ) );
    rtp->buffer_used = RTP_HDR_SIZE;

    ctn91xx_net_rx_skb( dev, skb, rx_len );
}

static void rtp_sink_read_data( vbuffer_t* vbuffer )
{
    rtp_state_t* rtp = &vbuffer->rtp_state;
    uint32_t bytes_left = RTP_SINK_BUFFER_SIZE - rtp->buffer_used;
    uint32_t num_ts_pkts = bytes_left / MPEG_TS_PKT_SIZE;
    uint8_t* unused_buffer_start = &rtp->buffer[rtp->buffer_used];

    if( num_ts_pkts > 0 ) {
#if HAS_MPEG_DMA
        uint32_t read_size = ctn91xx_mpeg_kernel_read(
                                 vbuffer, unused_buffer_start, MPEG_TS_PKT_SIZE*num_ts_pkts );
        rtp->buffer_used += read_size;
#endif
    }
}

static void rtp_data_ready( vbuffer_t* vbuffer )
{
    rtp_state_t* rtp = &vbuffer->rtp_state;
    uint32_t bytes_left = RTP_SINK_BUFFER_SIZE - rtp->buffer_used;

    if( bytes_left >= MPEG_TS_PKT_SIZE ) {
        rtp_sink_read_data( vbuffer );
    }

    bytes_left = RTP_SINK_BUFFER_SIZE - rtp->buffer_used;

    if( bytes_left < MPEG_TS_PKT_SIZE ) {
        rtp_send_packet( vbuffer );
    }
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
void ctn91xx_rtp_reader(void* work)
#else
void ctn91xx_rtp_reader(struct work_struct* work)
#endif
{
    rtp_state_t* rtp = container_of(work, rtp_state_t, reader);
    vbuffer_t* vbuffer = container_of(rtp, vbuffer_t, rtp_state);

    spin_lock_w_flags(&vbuffer->lock);

    if( !rtp->got_setup || !rtp->running ) {
        goto end;
    }

#if HAS_MPEG_DMA
    while( mpeg_data_ready( vbuffer ) ) {
        rtp_data_ready( vbuffer );
        if( !rtp->running ) {
            break;
        }
    }
#endif

end:
    spin_unlock_w_flags(&vbuffer->lock);
}


