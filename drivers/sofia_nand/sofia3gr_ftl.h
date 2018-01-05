/******************************************************************/
/* Copyright (C) 2014-2015 Fuzhou Rockchip Electronics Co., Ltd   */
/*******************************************************************
File    :   sofia3gr_ftl.h
Desc    :
Author  :   ZYF
Date    :   2015-05-15
Notes   :
Revision 1.00  2015/05/15 ZYF
Init file.
********************************************************************/
extern int nand_dev_suspend(void);
extern int nand_dev_resume(void);
extern void nand_dev_shutdown(void);
extern int init_nand_blk_dev(struct device *dev, unsigned int reg_addr);
extern struct nandc_info *gp_nandc_info;
extern void *nand_page_address(const struct page *page);
