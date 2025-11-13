#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

void Timer_Init(void);
uint16_t Timer_GetTIMCount(void);
uint16_t Timer_GetCount(void);
uint16_t Timer_GetCountWithReset(void);

#endif
