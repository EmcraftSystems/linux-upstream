#ifndef STM32_DSI_H
#define STM32_DSI_H

#define STM32_DSI_ORIENTATION_PORTRAIT	0
#define STM32_DSI_ORIENTATION_LANDSCAPE	1

struct stm32_dsi_panel {
	int (*init)(void *data, void (*writecmd)(void *data, int NbrParams, u8 *pParams), int orientation, bool te);
};


#endif /* STM32_DSI_H */
