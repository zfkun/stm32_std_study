#ifndef __BSP_MATH_H
#define __BSP_MATH_H

#ifndef _PI
#define _PI     3.1415926f
#endif

#ifndef _2PI
#define _2PI    6.2831853f
#endif

// 单位
// - 1弧度 = 180 / PI = 180 / 3.141592653589793 = 57.2958f
// - 1角度 = PI / 180 = 3.141592653589793 / 180 = 0.0174533f
// 换算
// - 角度  = 弧度 x 1弧度 = 弧度 x 57.2958f
// - 弧度  = 角度 x 1角度 = 角度 x 0.0174533f
//
#ifndef _1RAD
// 1弧度的角度值 (180 / PI = 57.2958f)
#define _1RAD   57.295779f
#endif

#ifndef _1DEG
// 1度的弧度值 (PI / 180 = 0.017453f)
#define _1DEG   0.017453f
#endif

float Math_Map(float value, float fromLow, float fromHigh, float toLow, float toHigh);
float Math_Normalize(float value, float min, float max);
unsigned int Math_Pow(unsigned int x, unsigned char y);

#endif
