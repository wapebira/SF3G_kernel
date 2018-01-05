/*                                                                                                                        
 ****************************************************************                                                         
 *                                                                                                                        
 *  Component: CIF driver                                                                                                 
 *                                                                                                                        
 *  Copyright (C) 2013 Intel Mobile Communications GmbH                                                                   
 *                                                                                                                        
 *  This program is free software: you can redistribute it and/or modify                                                  
 *  it under the terms of the GNU General Public License Version 2                                                        
 *  as published by the Free Software Foundation.                                                                         
 *                                                                                                                        
 *  This program is distributed in the hope that it will be useful,                                                       
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of                                                        
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                                                  
 *                                                                                                                        
 *  You should have received a copy of the GNU General Public License Version 2                                           
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.                                                  
 *                                                                                                                        
 ****************************************************************                                                         
 */ 
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>                                                                                        
#include <media/videobuf-dma-contig.h>                                                                                    
#include <linux/mm.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include "cif_isp20_regs.h"
#include "cif_isp20.h"
#include <linux/platform_device_pm.h>
#include "xgold-mipitune.h"
#include <linux/of_gpio.h>
#include <linux/platform_data/video-xgold_cif.h>                                                                          
#include "cif_isp20_img_src.h"                                                                                            
                                                                                                                          
                                                                                                                          
static int mipitune_dbg_level = 0x0F;                                                                                     
                                                                                                                          
#define MIPITUNE_DEBUG_ISR      (1<<5)                                                                                    
#define MIPITUNE_DEBUG_QUEUE    (1<<4)                                                                                    
#define MIPITUNE_DEBUG_FMT      (1<<3)                                                                                    
#define MIPITUNE_DEBUG_ENTER    (1<<2)                                                                                    
#define MIPITUNE_DEBUG_INFO     (1<<1)                                                                                    
#define MIPITUNE_DEBUG_ERROR    (1<<0)                                                                                    
                                                                                                                          
#define MIPITUNE_ERR_SOT        (0xf<<4)                                                                                  
#define MIPITUNE_ERR_SOT_SYNC   (0xf<<8)                                                                                  
#define MIPITUNE_ERR_EOT_SYNC   (0xf<<12)                                                                                 
#define MIPITUNE_ERR_CTRL       (0xf<<16)                                                                                 
#define MIPITUNE_ERR_PROTOCOL   (1<<20)                                                                                   
#define MIPITUNE_ERR_ECC2       (1<<21)                                                                                   
#define MIPITUNE_ERR_ECC1       (1<<22)                                                                                   
#define MIPITUNE_ERR_CS         (1<<23)                                                                                   
#define MIPITUNE_FRAME_END      (1<<24)                                                                                   
                                                                                                                          
#define MIPITUNE_ALL (MIPITUNE_ERR_SOT|MIPITUNE_ERR_SOT_SYNC|\
        MIPITUNE_ERR_EOT_SYNC|MIPITUNE_ERR_CTRL|MIPITUNE_ERR_PROTOCOL|\
        MIPITUNE_ERR_ECC2|MIPITUNE_ERR_ECC1|MIPITUNE_ERR_CS|MIPITUNE_FRAME_END)
                                                                                                                          
#define CIF_MIPITUNE_DPRINT(level,fmt,arg...) if (mipitune_dbg_level&level)printk("MIPITUNE " fmt,##arg)                  
                                                                                                                          
#define MIPITUNE_DEV_NAME "mipitune_dev"                                                                                  
                                                                                                                          
#define MIPITUNE_TARGET_PRI_PREVIEW	0                                                                                    
#define MIPITUNE_TARGET_PRI_CAPTURE	1                                                                                    
#define MIPITUNE_TARGET_SEC		2                                                                                          
#define MIPITUNE_TARGET_MAX		3                                                                                          
                                                                                                                          
#define MIPITUNE_PRI_SENSOR		0                                                                                          
#define MIPITUNE_SEC_SENSOR		1                                                                                          
                                                                                                                          
#define MIPITUNE_FMT_PREVIEW		0                                                                                          
#define MIPITUNE_FMT_CAPTURE            1                                                                                 
                                                                                                                          
/* Sensor Capture Formats */                                                                                              
#define MIPITUNE_PRI_PREVIEW_FMT	CIF_YVU420SP                                                                             
#define MIPITUNE_PRI_PREVIEW_FPS	30                                                                                       
                                                                                                                          
#define MIPITUNE_PRI_CAPTURE_FMT	CIF_JPEG                                                                                 
#define MIPITUNE_PRI_CAPTURE_FPS	10                                                                                       
                                                                                                                          
#define MIPITUNE_THUMBNAIL_FMT	CIF_RGB565                                                                                 
#define MIPITUNE_THUMBNAIL_WIDTH	112                                                                                      
#define MIPITUNE_THUMBNAIL_HEIGHT	84                                                                                     
                                                                                                                          
#define MIPITUNE_SEC_FMT		CIF_YVU420SP                                                                                   
#define MIPITUNE_SEC_FPS		30                                                                                             
                                                                                                                          
                                                                                                                          
//extern struct cif_isp20_device;                                                                                        
extern struct v4l2_fmtdesc output_formats[MAX_NB_FORMATS];
                                                                                                                          
static struct mipi_stat statistics;                                                                                       
                                                                                                                          
static unsigned int mipidphy1_tuning_value;                                                                               
static unsigned int mipidphy2_tuning_value;                                                                               
static unsigned int good_frame_count;                                                                                     
static unsigned int target_id=0;                                                                                          
static unsigned int sensor_id=0;                                                                                          
static unsigned int format_id=0;                                                                                          
static int (*orig_mipi_isr)(void*);                                                                                       
static int (*orig_isp_isr)(void*);                                                                                        
static unsigned int width;                                                                                                
static unsigned int height;                                                                                               
                                                                                                                          
static int mipituner_mipi_isr(void *cntxt)                                                                                
{                                                                                                                         
	struct cif_isp20_device *dev =                                                                                          
		(struct cif_isp20_device *)cntxt;                                                                                     
	//struct marvinconfig *marvin_config =                                                                                     
	//	&dev->xgold_hw.marvin_config;                                                                                          
	unsigned int mipi_mis =                                                                                                  
		cif_ioread32((dev->config.base_addr) + CIF_MIPI_MIS);                                                               
	//CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,"mi:0x%08x", mipi_mis);																		   
                                                                                                                          
	if(mipi_mis == MIPITUNE_FRAME_END)                                                                                       
		statistics.frame_end_no_errors++;                                                                                      
                                                                                                                          
	if(mipi_mis & MIPITUNE_FRAME_END)                                                                                        
		statistics.frame_end++;                                                                                                
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_SOT)                                                                                          
		statistics.err_sot++;                                                                                                  
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_SOT_SYNC)                                                                                     
		statistics.err_sot_sync++;                                                                                             
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_EOT_SYNC)                                                                                     
		statistics.err_eot_sync++;                                                                                             
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_CTRL)                                                                                         
		statistics.err_control++;                                                                                              
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_PROTOCOL)                                                                                     
		statistics.err_protocol++;                                                                                             
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_ECC2)                                                                                         
		statistics.err_ecc2++;                                                                                                 
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_ECC1)                                                                                         
		statistics.err_ecc1++;                                                                                                 
                                                                                                                          
	if(mipi_mis & MIPITUNE_ERR_CS)                                                                                           
		statistics.err_cs++;                                                                                                   
                                                                                                                          
	cif_iowrite32(mipi_mis, (dev->config.base_addr) + CIF_MIPI_ICR);                                                      
                                                                                                                          
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
static int mipituner_isp_isr(void *cntxt)                                                                                 
{                                                                                                                         
	struct cif_isp20_device *dev =                                                                                          
		(struct cif_isp20_device *)cntxt;                                                                                     
	//struct marvinconfig *marvin_config =                                                                                     
	//	&dev->xgold_hw.marvin_config;                                                                                          
	unsigned int isp_mis =                                                                                                   
		cif_ioread32(dev->config.base_addr + CIF_ISP_MIS);                                                                  
	//CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,"isp:0x%08x", isp_mis);																		   
                                                                                                                          
	if (isp_mis & CIF_ISP_PIC_SIZE_ERROR)                                                                                    
		statistics.err_isp_pic_size++;                                                                                         
                                                                                                                          
	if (isp_mis & CIF_ISP_DATA_LOSS)                                                                                         
		statistics.err_isp_data_loss++;                                                                                        
                                                                                                                          
	cif_iowrite32(~0, dev->config.base_addr + CIF_ISP_ICR);                                                               
                                                                                                                          
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
/*****************************************************************************************/                               
                                                                                                                          
static int mipitune_streamon(struct file *file, void *priv, enum v4l2_buf_type i)                                         
{                                                                                                                         
	int ret = 0;                                                                                                             
	struct cif_isp20_device* dev = video_get_drvdata(video_devdata(file));                                                  
	struct cif_isp20_strm_fmt request_strm_fmt;                                                                              
	enum cif_isp20_inp inp;                                                                                                  
	struct cif_isp20_csi_config csi_config;                                                                                  
                                                                                                                          
	CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "streamon\n");                                                                  
                                                                                                                          
	if (!width || !height) {                                                                                                 
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,                                                                              
			"output size not specified\n");                                                                                      
		return -EINVAL;                                                                                                        
	}                                                                                                                        
                                                                                                                          
	good_frame_count = 0;                                                                                                    
	memset(&statistics, 0, sizeof(statistics));                                                                              
                                                                                                                          
	if (sensor_id == MIPITUNE_PRI_SENSOR) {                                                                                  
		ret = cif_isp20_s_input(dev, 0);                                                                                       
		inp = CIF_ISP20_INP_CSI_0;                                                                                             
	} else {                                                                                                                 
		ret = cif_isp20_s_input(dev, 1);                                                                                       
		inp = CIF_ISP20_INP_CSI_1;                                                                                             
	}                                                                                                                        
	if (IS_ERR_VALUE(ret)) {                                                                                                 
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,                                                                              
			"can't set input\n");                                                                                                
		return ret;                                                                                                            
	}                                                                                                                        
                                                                                                                          
	if (format_id == MIPITUNE_FMT_PREVIEW) {                                                                                 
		if (sensor_id == MIPITUNE_PRI_SENSOR) { /*Primary Sensor*/                                                             
			request_strm_fmt.frm_fmt.pix_fmt = MIPITUNE_PRI_PREVIEW_FMT;                                                         
			request_strm_fmt.frm_fmt.width = width;                                                                              
			request_strm_fmt.frm_fmt.height = height;                                                                            
			request_strm_fmt.frm_intrvl.numerator = 1;                                                                           
			request_strm_fmt.frm_intrvl.denominator = MIPITUNE_PRI_PREVIEW_FPS;                                                  
		} else {                                                                                                               
			request_strm_fmt.frm_fmt.pix_fmt = MIPITUNE_SEC_FMT;                                                                 
			request_strm_fmt.frm_fmt.width = width;                                                                              
			request_strm_fmt.frm_fmt.height = height;                                                                            
			request_strm_fmt.frm_intrvl.numerator = 1;                                                                           
			request_strm_fmt.frm_intrvl.denominator = MIPITUNE_SEC_FPS;                                                          
		}                                                                                                                      
		//ret = cif_isp20_s_fmt_sp(dev, &request_strm_fmt, 0);                                                                   
		ret = cif_isp20_s_fmt(dev, CIF_ISP20_STREAM_SP, &request_strm_fmt, 0);                                                                   
	} else {                                                                                                                 
		request_strm_fmt.frm_intrvl.numerator = 1;                                                                             
		request_strm_fmt.frm_intrvl.denominator = MIPITUNE_PRI_CAPTURE_FPS;                                                    
		request_strm_fmt.frm_fmt.pix_fmt = MIPITUNE_THUMBNAIL_FMT;                                                             
		request_strm_fmt.frm_fmt.width = MIPITUNE_THUMBNAIL_WIDTH;                                                             
		request_strm_fmt.frm_fmt.height = MIPITUNE_THUMBNAIL_HEIGHT;                                                           
		//ret = cif_isp20_s_fmt_sp(dev, &request_strm_fmt, 0);                                                                   
		ret = cif_isp20_s_fmt(dev, CIF_ISP20_STREAM_SP, &request_strm_fmt, 0);                                                                   
		request_strm_fmt.frm_fmt.pix_fmt = MIPITUNE_PRI_CAPTURE_FMT;                                                           
		request_strm_fmt.frm_fmt.width = width;                                                                                
		request_strm_fmt.frm_fmt.height = height;                                                                              
		//ret |= cif_isp20_s_fmt_mp(dev, &request_strm_fmt, 0);                                                                  
		ret |= cif_isp20_s_fmt(dev, CIF_ISP20_STREAM_MP, &request_strm_fmt, 0);                                                                  
	}                                                                                                                        
	if (IS_ERR_VALUE(ret)) {                                                                                                 
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,                                                                              
			"can't set format\n");                                                                                               
		return ret;                                                                                                            
	}                                                                                                                        
                                                                                                                          
	ret = cif_isp20_pltfrm_g_csi_config(dev->dev,                                                                            
		sensor_id ? CIF_ISP20_INP_CSI_1 : CIF_ISP20_INP_CSI_0,                                                                 
		&request_strm_fmt,                                                                                                     
		&csi_config);                                                                                                          
	csi_config.dphy1 = mipidphy1_tuning_value;                                                                               
	csi_config.dphy2 = mipidphy2_tuning_value;                                                                               
	if (IS_ERR_VALUE(ret)) {                                                                                                 
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,                                                                              
			"can't retrieve CSI config from the device tree\n");                                                                 
		return ret;                                                                                                            
	}                                                                                                                        
                                                                                                                          
	ret = cif_isp20_pltfrm_s_csi_config(                                                                                     
		dev->dev, sensor_id ? CIF_ISP20_INP_CSI_1 : CIF_ISP20_INP_CSI_0,                                                       
		&request_strm_fmt, &csi_config);                                                                                       
	if (IS_ERR_VALUE(ret)) {                                                                                                 
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,                                                                              
			"can't set new CSI config\n");                                                                                       
		return ret;                                                                                                            
	}                                                                                                                        
                                                                                                                          
	ret = cif_isp20_streamon(dev,                                                                                                                                     
           format_id ? CIF_ISP20_STREAM_MP : CIF_ISP20_STREAM_SP); 																			
	if (IS_ERR_VALUE(ret)) {                                                                                                 
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,
			"streamon failed\n");                                                                                                
		return ret;                                                                                                            
	}                                                                                                                        
                                                                                                                          
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
static int mipitune_streamoff(struct file * file, void * priv, enum v4l2_buf_type i)                                      
{                                                                                                                         
	int ret = 0;                                                                                                             
	struct cif_isp20_device* dev = video_get_drvdata(video_devdata(file));                                                  
	//struct marvinconfig *marvin_config =                                                                                     
	//	&dev->xgold_hw.marvin_config;                                                                                          
                                                                                                                          
	CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO,                                                                                 
		"streamoff; total frames: %d, with errors: %d\n",                                                                      
		statistics.frame_end,                                                                                                  
		statistics.frame_end - statistics.frame_end_no_errors );                                                               
	 																		 
                                                                                                                          
	cif_isp20_streamoff(dev, format_id ? CIF_ISP20_STREAM_MP : CIF_ISP20_STREAM_SP);                                                                                    
	//cif_isp20_streamoff(dev, CIF_ISP20_STREAM_MP);                                                                                    
	if (IS_ERR_VALUE(ret)) {                                                                                                 
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,                                                                              
			"streamoff failed\n");                                                                                               
		return ret;                                                                                                            
	}                                                                                                                        
                                                                                                                          
	/* Read out the mipi config values used to be fully sure what was used. */                                               
	if (sensor_id == MIPITUNE_PRI_SENSOR) {  //Primary Sensor                                                                
		mipidphy1_tuning_value = cif_ioread32(                                                                                 
			(dev->config.base_addr)+CIF_MIPI_DPHY1_1);                                                                        
		mipidphy2_tuning_value = cif_ioread32(                                                                                 
			(dev->config.base_addr)+CIF_MIPI_DPHY1_2);                                                                        
	} else {	//Secondary Sensor                                                                                             
		mipidphy1_tuning_value = cif_ioread32(                                                                                 
			(dev->config.base_addr)+CIF_MIPI_DPHY2_1);                                                                        
		mipidphy2_tuning_value = cif_ioread32(                                                                                 
			(dev->config.base_addr)+CIF_MIPI_DPHY2_2);                                                                        
	}                                                                                                                        
                                                                                                                          
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
static int mipitune_g_ctrl(struct file *file, void *priv, struct v4l2_control *vc)                                        
{                                                                                                                         
	switch(vc->id){                                                                                                          
	case V4L2_CID_CIFISP_MIPITUNER_DPHY1_VALUE:                                                                              
		vc->value = mipidphy1_tuning_value;                                                                                    
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "g_ctrl DPHY1 0x%08x\n", mipidphy1_tuning_value);                             
		break;                                                                                                                 
	case V4L2_CID_CIFISP_MIPITUNER_DPHY2_VALUE:                                                                              
		vc->value = mipidphy2_tuning_value;                                                                                    
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "g_ctrl DPHY2 0x%08x\n", mipidphy2_tuning_value);                             
		break;                                                                                                                 
	default:                                                                                                                 
		return -EINVAL;                                                                                                        
	}                                                                                                                        
                                                                                                                          
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
static int mipitune_s_fmt(struct file *file, void *priv, struct v4l2_format *f)                                           
{                                                                                                                         
	width = f->fmt.pix.width;                                                                                                
	height = f->fmt.pix.height;                                                                                              
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
/* */                                                                                                                     
static int mipitune_s_ctrl(struct file *file, void *priv, struct v4l2_control *vc)                                        
{                                                                                                                         
        switch(vc->id){                                                                                                   
        case   V4L2_CID_CIFISP_MIPITUNER_DPHY1_VALUE:                                                                     
                mipidphy1_tuning_value = vc->value;                                                                       
                CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "s_ctrl: DPHY1=0x%08x\n", mipidphy1_tuning_value);               
                break;                                                                                                    
        case   V4L2_CID_CIFISP_MIPITUNER_DPHY2_VALUE:                                                                     
                mipidphy2_tuning_value = vc->value;                                                                       
                CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "s_ctrl: DPHY2=0x%02x\n", mipidphy2_tuning_value);               
                break;                                                                                                    
        case   V4L2_CID_CIFISP_MIPITUNER_SEL_TARGET:                                                                      
                target_id = vc->value;                                                                                    
                                                                                                                          
                if (target_id >= MIPITUNE_TARGET_MAX) {                                                                   
		    CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "s_ctrl: unknown target id.\n");                                          
                    return -EINVAL;                                                                                       
                } else {                                                                                                  
                    switch (target_id) {                                                                                  
                        case MIPITUNE_TARGET_PRI_PREVIEW:                                                                 
                            sensor_id = MIPITUNE_PRI_SENSOR;                                                              
                            format_id = MIPITUNE_FMT_PREVIEW;                                                             
                            break;                                                                                        
                        case MIPITUNE_TARGET_PRI_CAPTURE:                                                                 
                            sensor_id = MIPITUNE_PRI_SENSOR;                                                              
                            format_id = MIPITUNE_FMT_CAPTURE;                                                             
                            break;                                                                                        
                        case MIPITUNE_TARGET_SEC:                                                                         
                            sensor_id = MIPITUNE_SEC_SENSOR;                                                              
                            format_id = MIPITUNE_FMT_PREVIEW;                                                             
                            break;                                                                                        
                        default:                                                                                          
			    CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "s_ctrl: Unknown parameters\n");                                        
                            return -EINVAL;                                                                               
                            break;                                                                                        
                    }                                                                                                     
                }                                                                                                         
                CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "s_ctrl: Current mode is (%d, %d).\n", sensor_id, format_id);    
                break;                                                                                                    
        default:                                                                                                          
                CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "s_ctrl: Unknown parameters\n");                                 
                return -EINVAL;                                                                                           
        }                                                                                                                 
                                                                                                                          
        return 0;                                                                                                         
}                                                                                                                         
                                                                                                                          
static long mipitune_ioctl_default(struct file *file,                                                                     
                                void *fh,                                                                                 
                                bool valid_prio,                                                                          
                                unsigned int cmd,                                                                         
                                void *arg)                                                                                
{                                                                                                                         
        if(cmd != CIFMIPITUNER_IOC_G_STATS)                                                                               
                return -EINVAL;                                                                                           
                                                                                                                          
        memcpy(arg, &statistics, sizeof(struct mipi_stat));                                                               
                                                                                                                          
        return 0;                                                                                                         
}                                                                                                                         
                                                                                                                          
/* Main path video device IOCTLs*/                                                                                        
static const struct v4l2_ioctl_ops mipitune_ioctl = {                                                                     
 .vidioc_streamon       = mipitune_streamon,                                                                              
 .vidioc_streamoff      = mipitune_streamoff,                                                                             
 .vidioc_g_ctrl         = mipitune_g_ctrl,                                                                                
 .vidioc_s_ctrl         = mipitune_s_ctrl,                                                                                
 .vidioc_s_fmt_vid_cap  = mipitune_s_fmt,                                                                                 
 .vidioc_s_fmt_vid_overlay = mipitune_s_fmt,                                                                              
 .vidioc_default        = mipitune_ioctl_default,                                                                         
};                                                                                                                        
                                                                                                                          
/* =============================================================================== */                                     
                                                                                                                          
static int mipitune_open(struct file *file)                                                                               
{                                                                                                                         
	struct cif_isp20_device * dev = video_get_drvdata(video_devdata(file));                                                 
                                                                                                                          
	CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "open\n");                                                                      
                                                                                                                          
	if ((dev->sp_stream.state != CIF_ISP20_STATE_DISABLED) ||                                                                
	    (dev->mp_stream.state != CIF_ISP20_STATE_DISABLED)) {                                                                
		CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_ERROR,                                                                              
			"mipituner cannot be used while CIF ISP is already in use\n");                                                       
		return -EBUSY;                                                                                                         
	}                                                                                                                        
                                                                                                                          
	//cif_isp20_init_sp(dev);                                                                                                  
	//cif_isp20_init_mp(dev); 
	cif_isp20_init(dev,CIF_ISP20_STREAM_SP);
	cif_isp20_init(dev,CIF_ISP20_STREAM_MP);
                                                                                                                          
	orig_mipi_isr = cif_isp20_pltfrm_irq_g_isr(dev->dev, CIF_ISP20_IRQ_MIPI);                                                
	orig_isp_isr = cif_isp20_pltfrm_irq_g_isr(dev->dev, CIF_ISP20_IRQ_ISP);                                                  
	width = 0;                                                                                                               
	height = 0;                                                                                                              
                                                                                                                          
	cif_isp20_pltfrm_irq_register_isr(                                                                                       
		dev->dev,                                                                                                              
		CIF_ISP20_IRQ_MIPI,                                                                                                    
		mipituner_mipi_isr,                                                                                                    
		dev);                                                                                                                  
	cif_isp20_pltfrm_irq_register_isr(                                                                                       
		dev->dev,                                                                                                              
		CIF_ISP20_IRQ_ISP,                                                                                                     
		mipituner_isp_isr,                                                                                                     
		dev);                                                                                                                  
                                                                                                                          
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
/* =============================================================================== */                                     
                                                                                                                          
static int mipitune_release(struct file *file)                                                                            
{                                                                                                                         
	struct cif_isp20_device * dev = video_get_drvdata(video_devdata(file));                                                 
                                                                                                                          
	if (IS_ERR_VALUE(cif_isp20_pltfrm_irq_register_isr(                                                                      
		dev->dev,                                                                                                              
		CIF_ISP20_IRQ_MIPI,                                                                                                    
		orig_mipi_isr,                                                                                                         
		dev)))                                                                                                                 
		BUG();                                                                                                                 
	if (IS_ERR_VALUE(cif_isp20_pltfrm_irq_register_isr(                                                                      
		dev->dev,                                                                                                              
		CIF_ISP20_IRQ_ISP,                                                                                                     
		orig_isp_isr,                                                                                                          
		dev)))                                                                                                                 
		BUG();                                                                                                                 
	cif_isp20_reset_csi_configs(dev->dev,                                                                                    
		sensor_id ? CIF_ISP20_INP_CSI_1 : CIF_ISP20_INP_CSI_0);                                                                
	if (IS_ERR_VALUE(cif_isp20_release(dev, CIF_ISP20_STREAM_MP)))                                                                    
		BUG();                                                                                                                 
	if (IS_ERR_VALUE(cif_isp20_release(dev, CIF_ISP20_STREAM_SP)))                                                                    
		BUG();                                                                                                                 
                                                                                                                          
                                                                                                                          
	return 0;                                                                                                                
}                                                                                                                         
                                                                                                                          
/* =============================================================================== */                                     
                                                                                                                          
struct v4l2_file_operations  mipitune_fops = {                                                                            
    .open	= mipitune_open,                                                                                               
	 .release = mipitune_release,                                                                                            
    .ioctl	= video_ioctl2                                                                                                 
};                                                                                                                        
                                                                                                                          
int register_mipitune_device(struct cif_isp20_device *xgold_v4l2)
{                                                                                                                         
    int ret = 0;                                                                                                          
    struct video_device *vdev_mipitune;                                                                                   
                                                                                                                          
    CIF_MIPITUNE_DPRINT(MIPITUNE_DEBUG_INFO, "register device\n"); 

    vdev_mipitune = video_device_alloc();
                                                                                                                          
    if (vdev_mipitune){
        vdev_mipitune->release = video_device_release;
        strlcpy(vdev_mipitune->name, MIPITUNE_DEV_NAME, sizeof(vdev_mipitune->name));
        vdev_mipitune->vfl_type =  V4L2_CAP_VIDEO_CAPTURE;                                                                
        vdev_mipitune->fops = &mipitune_fops;                                                                             
        video_set_drvdata(vdev_mipitune, xgold_v4l2);                                                                     
        vdev_mipitune->minor = -1;                                                                                        
        vdev_mipitune->ioctl_ops = &mipitune_ioctl;                                                                       
        vdev_mipitune->v4l2_dev = &xgold_v4l2->v4l2_dev;                                                                  
                                                                                                                          
        //ret = video_register_device(vdev_mipitune, VFL_TYPE_GRABBER, -1);                                                 
        ret = video_register_device(vdev_mipitune, VFL_TYPE_GRABBER, 4);  //"/dev/video4 for tuner"                                             
                                                                                                                          
        if (ret < 0) {                                                                                                    
        	dev_err(&(vdev_mipitune->dev),"could not register mipituner \n");                                                
            video_device_release(vdev_mipitune);                                                                          
        }                                                                                                                 
    }                                                                                                                     
    else{                                                                                                                 
        ret = -ENOMEM;                                                                                                    
    }                                                                                                                     
                                                                                                                          
    return ret;                                                                                                           
}             
EXPORT_SYMBOL(register_mipitune_device);