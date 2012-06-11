#ifndef CTN91XX_STRUCTS_H
#define CTN91XX_STRUCTS_H

#define CTN91XX_MAGIC 0x1B7C0004

/* Structure of the interrupt data on the card */
typedef struct cablecard_interrupt {
    uint8_t origin; /* offset 0 */
    uint8_t control_byte; /* offset 1 */
    uint16_t length; /* offset 2 */
    uint8_t type; /* offset 4 */
    uint8_t interrupt_set; /* offset 5 */
    uint16_t reserved; /* offset 6 */
    uint8_t seqno; /* offset 8 */
} cablecard_interrupt_t;


struct ctn91xx_dev;

typedef struct ctn91xx_dev ctn91xx_dev_t;

typedef struct ctn91xx_dev_fd_state {
    ctn91xx_dev_t* dev;
    int minor;
} ctn91xx_dev_fd_state_t;

#define SIZEOF_EVENT_HEADER 6

struct ctn91xx_event {
    uint8_t type;
    uint16_t length;
    uint8_t origin;
    uint8_t control_byte;
    uint8_t seqno;
    uint8_t should_queue;

    struct list_head list;
};
typedef struct ctn91xx_event ctn91xx_event_t;

typedef struct {
    uint32_t last_cc;
    uint32_t discontinuities;
    uint32_t count;
    uint32_t out_of_sync;
} pid_cc_t;

typedef struct {
    uint8_t remote_hw_addr[6];
    uint8_t local_hw_addr[6];
    uint32_t remote_ip_addr;
    uint32_t local_ip_addr;
    uint16_t remote_port;
    uint16_t local_port;

    uint8_t got_setup;
    uint8_t running;

    uint8_t* buffer;
    uint32_t buffer_used;

    uint32_t start_rtp_time;
    uint32_t start_seq;
    uint32_t seq_no;
    uint32_t ip_seq_no;
    ktime_t prev_time;
    uint32_t rtp_time_delta;
    uint32_t ssrc;

    struct work_struct reader;

} rtp_state_t;

typedef struct {
    uint8_t csrc_len:4;
    uint8_t extension:1;
    uint8_t padding:1;
    uint8_t version:2;

    uint8_t payload:7;
    uint8_t marker:1;

    uint16_t seq_no;
    uint32_t timestamp;
    uint32_t ssrc;
} rtp_header_t;

struct vbuffer {
    uint32_t npages;
    uint8_t** buffers;
    dma_addr_t* dma_addrs;
    uint32_t read_idx;
    uint32_t write_idx;
    uint32_t notify_idx;
    struct page** pages;
    spinlock_t lock;

    /**
     * array of size npages indicating how many
     * users are using the page.  greater than 0
     * means the page has been mapped
     */
    uint8_t* lock_cnt;
    uint8_t* dropped;

    /**
     * array of size npages telling the current
     * number of bytes in each page
     */
    uint16_t* remaining;
    uint16_t* sizes;

    /**
     * Buffer for storing and copying any headers used in MPEG_SEND
     * vmalloc'd
     */
    uint8_t* header_buffer;
    struct page* header_buffer_page;

    int tuner_index;
    uint32_t packet_count;

    ctn91xx_dev_t* dev;

#if USE_PCI
    rtp_state_t rtp_state;
#endif
};

typedef struct vbuffer vbuffer_t;

typedef struct {
    /* i2c stuff */
    uint32_t read_index;
    uint32_t read_len;
    uint32_t write_index;
    uint8_t state;
    uint8_t error;
    i2c_write_read_t msg;
} ctn91xx_i2c_state_t;

typedef struct {
    struct mutex mutex;
    wait_queue_head_t wait_queue;
    spit_write_read_t msg;
    uint8_t* write_buffer;
    uint8_t* read_buffer;
    uint8_t error;
    uint8_t done;
} ctn91xx_spit_state_t;

typedef struct {
    ctn91xx_dev_t* ctn91xx_dev;
    struct net_device_stats stats;
    spinlock_t lock;
    int tx_outstanding;
    struct sk_buff* tx_skb;
} ctn91xx_net_priv_t;

typedef struct {
    uint8_t value;
    uint8_t is_input;
    uint8_t hw_is_input;
} ctn91xx_gpio_t;

typedef struct {

    uint8_t next_cgms_value;
    uint8_t cgms_value;
    uint8_t in_class_cgms;
    uint8_t checksum;
    uint8_t cont;
    ktime_t last_event_time;

} ctn91xx_scte21_state_t;


typedef struct {
    //these three are used by hardware
    uint32_t            ctrl;
    uint32_t            dma_addr; //physical address
    uint32_t            nxt_addr; //physical address

    dma_addr_t          paddr; //this descriptors physical address
    void*               priv;
    uint8_t*            virt;
    struct list_head    list;
} dma_desc_t;

typedef struct {
    ctn91xx_dev_t* dev;

    struct list_head desc_pool;
    spinlock_t desc_pool_lock;
    int num_tx_desc_active;
    int num_rx_desc_active;

    uint8_t*            buffers;
    uint64_t*           phys_addrs;

    struct net_device*  netdev;
    uint32_t num_interrupts;

} ctn91xx_eth_t;

struct ctn91xx_dev {
    int magic;
    void __iomem* base;
    unsigned long hw_reg_base;
    void __iomem* translation_base;
    unsigned long translation_hw_reg_base;

    void __iomem* mpeg_filter_dma_base;
    void __iomem* mpeg_dma_base;
    void __iomem* system_control_base;
    void __iomem* msg_base;
    void __iomem* timer_reg_base;


#if USE_PCI
    struct pci_dev* pdev;
#endif

    //pci side
    uint8_t board_number;
    int board_booting;
    int face_present;
    int ticks_per_us;
    int irq;
    int int_enable;
    struct class* class;

    //leon side
    int slot_number;
    int is_noah;
    int face_attached;
    int face_supports_reset;
    int face_reset_caps_detected;

    struct cdev ctrl_cdev;
    struct cdev mpeg_cdev;

    uint8_t card_status;
    uint16_t release_version;
    uint16_t chip_version;
    uint32_t board_version; //deprecated

    uint32_t which_proc;

    uint32_t always_scard;
    uint32_t dma_timeout;

    /* event queue */
    spinlock_t event_queue_lock;
    struct list_head event_queue;
    wait_queue_head_t event_waitqueue;
    int event_user_cnt;

    /* delayed event queue */
    spinlock_t delayed_event_queue_lock;
    struct list_head delayed_event_queue;
    struct work_struct delayed_event_worker;
    struct work_struct delayed_slot_number_worker;
    struct delayed_work gpio_poll;

    struct mutex fd_mutex;

    spinlock_t device_lock;
    spinlock_t isr_lock;

    /* mpeg stuff */
    vbuffer_t mpeg_buffer[NUM_MPEG_DEVICES];
    uint32_t mpeg_user_cnt[NUM_MPEG_DEVICES];
    wait_queue_head_t mpeg_wait_queues[NUM_MPEG_DEVICES];


    /* net device and msg buffer stuff */
    struct net_device* net_dev;
    wait_queue_head_t msg_buffer_wait_queue;
    uint8_t* rpc_buffer;

    /* stats */
    ctn91xx_stats_t stats;

#if CHECK_NOTIFY_CC
    pid_cc_t pid_cc_notify[6][0x1fff];
#endif

};

#define WRITE_LOCK() spin_lock_w_flags(&dev->device_lock)
#define WRITE_RELOCK() spin_relock_w_flags(&dev->device_lock)
#define WRITE_UNLOCK() spin_unlock_w_flags(&dev->device_lock)

#endif
