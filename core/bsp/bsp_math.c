
#include "bsp_math.h"

/**
 * 幂运算
 * 
 * @param x 输入值
 * @param y 幂次
 */
unsigned int Math_Pow(unsigned int x, unsigned char y)
{
    if (x == 0 || y == 0)
    {
        return 1;
    }

    unsigned int res = 1;
    while (y--)
    {
        res *= x;
    }

    return res;
}

/**
 * 数值重映射
 * 
 * @param x 输入值
 * @param fromLow 输入最小值
 * @param fromHigh 输入最大值
 * @param toLow 输出最小值
 * @param toHigh 输出最大值
 */
float Math_Map(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
	if (fromLow == fromHigh) {
		return (toLow == toHigh) ? toLow : (value > fromLow ? toHigh : toLow);
	}
	else if (value <= fromLow) {
		return toLow;
	}
	else if (value >= fromHigh) {
		return toHigh;
	}

    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

/**
 * 归一化
 * 
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 */
float Math_Normalize(float value, float min, float max) {
	if (min >= max) {
		return 0.0f;
	}
	else if (value <= min) {
		return 0.0f;
	}
	else if (value >= max) {
		return 1.0f;
	}

    return (value - min) / (max - min);
}
