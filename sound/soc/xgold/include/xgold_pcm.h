/*
 * Component: XGOLD PCM header file
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 */

#ifndef __XGOLD_PCM_H__
#define __XGOLD_PCM_H__

enum xgold_pcm_stream_type {
	STREAM_PLAY = 0,
	STREAM_PLAY2,
	STREAM_REC,
	HW_PROBE_A,
	HW_PROBE_B,
	NR_STREAM
};

struct xgold_pcm {
	struct device *dev;
	struct dsp_audio_device *dsp;
	bool dma_mode;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;
	unsigned int path_select;
};

struct xgold_runtime_data {
	struct xgold_pcm *pcm;
	struct snd_pcm_substream *stream;
	enum xgold_pcm_stream_type stream_type;
	unsigned short *hwptr;
	unsigned int hwptr_done;
	unsigned int periods;
	unsigned int period_size_bytes;
	/* DMA stream */
	struct scatterlist *dma_sgl;
	struct dma_chan *dmach;
	dma_cookie_t dma_cookie;
	spinlock_t lock;
};

#endif /* __XGOLD_PCM_H__ */
