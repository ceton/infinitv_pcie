#ifndef CTN91XX_DRIVER_H
#define CTN91XX_DRIVER_H

#include "ctn91xx_kal.h"

void ctn91xx_print_compliation(void);

int ctn91xx_register_ctrl_device( ctn91xx_dev_t* dev );
void ctn91xx_unregister_ctrl_device( ctn91xx_dev_t* dev );

ctn91xx_dev_t* ctn91xx_lookup_dev_from_file(struct inode* inode, struct file* file);
void ctn91xx_cleanup_dev_from_file(struct inode* inode, struct file* file);
int ctn91xx_lookup_minor_from_file(struct inode* inode, struct file* file);

void ctn91xx_destroy_board(ctn91xx_dev_t* dev);

#endif

