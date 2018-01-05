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
                                                                               
#ifndef _XGOLD_MIPITUNE_H                                                      
#define _XGOLD_MIPITUNE_H                                                      
                                                                               
#define V4L2_CID_CIFISP_MIPITUNER_DPHY1_VALUE (V4L2_CID_PRIVATE_BASE + 0)      
#define V4L2_CID_CIFISP_MIPITUNER_DPHY2_VALUE (V4L2_CID_PRIVATE_BASE + 1)      
                                                                               
#define V4L2_CID_CIFISP_MIPITUNER_SEL_TARGET (V4L2_CID_PRIVATE_BASE + 2)       
                                                                               
struct mipi_stat{                                                              
        unsigned int err_sot;                                                  
        unsigned int err_sot_sync;                                             
        unsigned int err_eot_sync;                                             
        unsigned int err_control;                                              
        unsigned int err_protocol;                                             
        unsigned int err_ecc2;                                                 
        unsigned int err_ecc1;                                                 
        unsigned int err_cs;                                                   
        unsigned int frame_end_no_errors;                                      
        unsigned int frame_end;                                                
		  unsigned int err_isp_pic_size;                                            
		  unsigned int err_isp_data_loss;                                           
};                                                                             
                                                                               
#define CIFMIPITUNER_IOC_G_STATS \                                           
        _IOR('v', BASE_VIDIOC_PRIVATE + 0, struct mipi_stat)                   


int register_mipitune_device(struct cif_isp20_device *xgold_v4l2);
#endif                                                                         
                                                                               