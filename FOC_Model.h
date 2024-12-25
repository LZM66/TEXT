#pragma once
#ifndef RTW_HEADER_FOC_Model_h_
#define RTW_HEADER_FOC_Model_h_
#ifndef FOC_Model_COMMON_INCLUDES_
#define FOC_Model_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)  ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm,val)  ((rtm)->errorStatus = (val))
#endif

typedef struct tag_RTM RT_MODEL;

typedef struct {
	real32_T Integrator_Dstate;
	real32_T Integrator_Prev_U;
	uint32_T Speed_loop_PREV_T;
	int8_T Integrator_PrevResetState;
	uint8_T Integrator_SYSTEM_ENABLE;
	boolean_T Speed_loop_RESET_ELAPS_T;
}DT_Speed_loop;

typedef struct {
	real32_T DSMO_eialpha;
	real32_T DSMO_eibeta;
	real32_T DSMO_ealpha;
	real32_T DSMO_ebeta;
}DT_SMO;

typedef struct {
	real32_T LPF_We;
}DT_LPF;

typedef struct {
	real32_T etheta;
	real32_T Integrator_Dstate;
}DT_PLL;

typedef struct {
	DT_PLL DPLL;
	DT_LPF DLPF;
	DT_SMO DSMO;
	real32_T ewe;
}DT_SMO_C;

typedef struct {
	DT_SMO_C DSMO_C;
	real_T cnt;
	real32_T Currloop_theta;
	real32_T Currloop_SMO_theta;
	real32_T Currloop_iq;
	real32_T ZReset;
	real32_T Motor_state;

	real32_T Integrator_idAdd;
	real32_T Integrator_iqAdd;
	real32_T Currloop_If4_theta;
	real32_T Currloop_If4_UDelay_iq;
	real32_T Currloop_If3_welimit;
	real32_T Currloop_If3_theta;

	uint32_T temporalCounter_i1;
	int8_T Currloop_If3_DiscreteTimeInte_i;
	int8_T Currloop_If3_DiscreteTimeInte_b;
	uint8_T is_active_c3_FOC_Model;
	uint8_T is_c3_FOC_Model;
}DT_Curr_loop;

typedef struct {
	DT_Curr_loop DCurr_loop;
	DT_Speed_loop DSpeed_loop;
	real_T ResetSignal;
	real_T ResetSignal_Buffer;
	real32_T SVPWM[3];
	real32_T iqref;
	real32_T iqref_speedloop;
	real32_T speedfd;
	
}DT;
/**/
typedef struct {
	real32_T ia;
	real32_T ib;
	real32_T ic;
	real32_T speedRef;
	real32_T Motor_OnOff;
	real32_T Udc;
}IN_U;

typedef struct {
	real32_T PWM1;
	real32_T PWM2;
	real32_T PWM3;
}OUT_PWM;

typedef struct {
	real32_T L;
	real32_T Rs;
	real32_T Pn;
}Motor_Ca;

typedef struct {
	real32_T K;
	real32_T M;
	real32_T LPFfilter;
	real32_T pll_ki;
	real32_T pll_kp;
	
}SMO_Ca;

typedef struct {
	real32_T curr_ki;
	real32_T curr_kp;
}Curr_Ca;

typedef struct {
	real32_T spd_ki;
	real32_T spd_kp;
}Speed_Ca;

struct tag_RTM{
	const char_T *volatile errorStatus;
	struct{
		uint32_T clockTick1;
		struct {
			uint8_T TID0_1;
		}RateInteraction;
	}Timing;
};

extern DT rtDT;
extern IN_U rtOUT_U;
extern OUT_PWM rtOUT_PWM;

extern void FOC_Model_Init(void);
extern void FOC_Model_step0(void);
extern void FOC_Model_step1(void);

typedef struct {
	struct {
		uint32_T wordH;
		uint32_T wordL;
	} words;
} BigEndianIEEEDouble;

typedef struct {
	struct {
		uint32_T wordL;
		uint32_T wordH;
	} words;
} LittleEndianIEEEDouble;

typedef struct {
	union {
		real32_T wordLreal;
		uint32_T wordLuint;
	} wordL;
} IEEESingle;
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;

extern Motor_Ca Motor;
extern SMO_Ca SMO;
extern Curr_Ca CurrKpKi;
extern Speed_Ca SpeedKpKi;
extern RT_MODEL* const rtM;

static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);

static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);

#endif
