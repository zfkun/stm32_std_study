#ifndef __BSP_MATH_H
#define __BSP_MATH_H

float Math_Map(float value, float fromLow, float fromHigh, float toLow, float toHigh);
float Math_Normalize(float value, float min, float max);
unsigned int Math_Pow(unsigned int x, unsigned char y);

#endif
