#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "struct_typedef.h"
extern void bsp_buzzer_set_freq(uint32_t freq);
extern void bsp_buzzer_off(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BSP_BUZZER_H_ */
