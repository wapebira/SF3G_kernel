#ifndef _V4L2_CONTROLS_ROCKCHIP_H
#define _V4L2_CONTROLS_ROCKCHIP_H

#include <linux/videodev2.h>
#include <media/v4l2-controls_intel.h>

#define RK_XGOLD_CAMERA_STRLEN   32

struct rk_xgold_camera_module_info_s {
	char module_name[RK_XGOLD_CAMERA_STRLEN];
	char len_name[RK_XGOLD_CAMERA_STRLEN];
};

#define RK_VIDIOC_CAMERA_MODULEINFO _IOWR('v', BASE_VIDIOC_PRIVATE + 10, struct rk_xgold_camera_module_info_s)
#endif
