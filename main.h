#pragma once
#include <stdio.h>
#include "Include/arm_math.h"

#define NUM 100

typedef struct {
	float Buff[NUM];
	float SumV;
	uint16_t index;
	uint16_t InvNUM;
}MAF_Filter;