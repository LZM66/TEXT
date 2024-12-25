#define _CRT_SECURE_NO_WARNINGS 1
#include <FOC_Model.h>
#include <math.h>
#include <rtwtypes.h>
#include <float.h>
#include <stddef.h>
/*载波频率：20kHz/0.00005s*/
#define IN_AlignStage                  ((uint8_T)1U)
#define IN_IDLE                        ((uint8_T)2U)
#define IN_OpenStage                   ((uint8_T)3U)
#define IN_RunStage                    ((uint8_T)4U)
#define IN_ThetaAlign                  ((uint8_T)5U)
#define NumBitsPerChar                 8U

Motor_Ca Motor = {
	0.000789, 
	1.0, 
	19.0
};
SMO_Ca SMO = {
	1520.0, 
	33000.0, 
	0.003, 
	4400.0, 
	1050.0
};
Curr_Ca CurrKpKi = {
	25.0, 
	0.9
};
Speed_Ca SpeedKpKi = {
	1.4,//0.08 
	0.025//0.0025
};

DT rtDT;
IN_U rtIN;
OUT_PWM rtOUT;

static RT_MODEL rtM_;
RT_MODEL* const rtM = &rtM_;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
/*初始化代码所需的NaN*/
static real_T rtGetNaN(void)
{
	size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
	real_T nan = 0.0;
	if (bitsPerReal == 32U) {
		nan = rtGetNaNF();
	}
	else {
		union {
			LittleEndianIEEEDouble bitVal;
			real_T fltVal;//初始化值？
		} tmpVal;

		tmpVal.bitVal.words.wordH = 0xFFF80000U;
		tmpVal.bitVal.words.wordL = 0x00000000U;
		nan = tmpVal.fltVal;
	}
	return nan;
}
/*初始化代码所需的NaNF*/
static real32_T rtGetNaNF(void)
{
	IEEESingle nanF = { { 0.0F } };

	nanF.wordL.wordLuint = 0xFFC00000U;
	return nanF.wordL.wordLreal;
}
/*初始化代码所需的Inf、MinusInf、NaN*/
static void rt_InitInfAndNaN(size_t realSize)
{
	(void)(realSize);
	rtNaN = rtGetNaN();
	rtNaNF = rtGetNaNF();
	rtInf = rtGetInf();
	rtInfF = rtGetInfF();
	rtMinusInf = rtGetMinusInf();
	rtMinusInfF = rtGetMinusInfF();
}
/*测试双精度值是否为无穷大*/
static boolean_T rtIsInf(real_T value)
{
	return (boolean_T)((value == rtInf || value == rtMinusInf) ? 1U : 0U);
}
/*测试单精度值是否为无穷大*/
static boolean_T rtIsInfF(real32_T value)
{
	return (boolean_T)(((value) == rtInfF || (value) == rtMinusInfF) ? 1U : 0U);
}
/*测试双精度值是否不是数字*/
static boolean_T rtIsNaN(real_T value)
{
	boolean_T result = (boolean_T)0;
	size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
	if (bitsPerReal == 32U) {
		result = rtIsNaNF((real32_T)value);
	}
	else {
		union {
			LittleEndianIEEEDouble bitVal;
			real_T fltVal;//初始化值？
		} tmpVal;

		tmpVal.fltVal = value;
		result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
			((tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
				(tmpVal.bitVal.words.wordL != 0)));
	}

	return result;
}
/*测试单精度值是否不是数字*/
static boolean_T rtIsNaNF(real32_T value)
{
	IEEESingle tmp;//初始化值？
	tmp.wordL.wordLreal = value;
	return (boolean_T)((tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
		(tmp.wordL.wordLuint & 0x007FFFFF) != 0);
}
/*初始化代码所需的Inf*/
static real_T rtGetInf(void)
{
	size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
	real_T inf = 0.0;
	if (bitsPerReal == 32U) {
		inf = rtGetInfF();
	}
	else {
		union {
			LittleEndianIEEEDouble bitVal;
			real_T fltVal;//初始化值？
		} tmpVal;

		tmpVal.bitVal.words.wordH = 0x7FF00000U;
		tmpVal.bitVal.words.wordL = 0x00000000U;
		inf = tmpVal.fltVal;
	}

	return inf;
}
/*初始化代码所需的InfF*/
static real32_T rtGetInfF(void)
{
	IEEESingle infF;//初始化值？
	infF.wordL.wordLuint = 0x7F800000U;
	return infF.wordL.wordLreal;
}
/*初始化代码所需的MinusInf*/
static real_T rtGetMinusInf(void)
{
	size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
	real_T minf = 0.0;
	if (bitsPerReal == 32U) {
		minf = rtGetMinusInfF();
	}
	else {
		union {
			LittleEndianIEEEDouble bitVal;
			real_T fltVal;//初始化值？
		} tmpVal;

		tmpVal.bitVal.words.wordH = 0xFFF00000U;
		tmpVal.bitVal.words.wordL = 0x00000000U;
		minf = tmpVal.fltVal;
	}

	return minf;
}
/*初始化代码所需的MinusInfF*/
static real32_T rtGetMinusInfF(void)
{
	IEEESingle minfF;//初始化值？
	minfF.wordL.wordLuint = 0xFF800000U;
	return minfF.wordL.wordLreal;
}


/*速度环初始化*/
static void Speedloop_Init(DT_Speed_loop* localDT)
{
	localDT->Integrator_PrevResetState = 2;
}
/*速度环使能*/
static void Speedloop_Enable(DT_Speed_loop* localDT)
{
	localDT->Speed_loop_RESET_ELAPS_T = true;
	localDT->Integrator_SYSTEM_ENABLE = 1U;
}
/*速度环*/
/*
rtM:时间相关结构体
rtIN_U_Speedfd：实际转速（rpm）
rtIN_U_Speedref:设定转速（rpm）
rtIN_ResetSignal：速度环启停标志，1：启动；0：停止

rtOUT_iq_ref：速度环输出，期望iq
localDT:速度环结构体参数指针
*/
static void Speedloop(RT_MODEL* const rtM, real32_T rtIN_U_Speedfd,real32_T rtIN_U_Speedref,
	real_T rtIN_ResetSignal, real32_T* rtOUT_iq_ref, DT_Speed_loop* localDT)
{
	real32_T Integrator;
	real32_T SpeedloopOUT;
	real32_T Speederr;
	uint32_T Speed_loop_ELAPS_T;
	int8_T tmp;
	int8_T tmp_0;

	if (localDT->Speed_loop_RESET_ELAPS_T)
	{
		Speed_loop_ELAPS_T = 0U;
	}
	else 
	{
		//Speed_loop_ELAPS_T = rtM->Timing.clockTick1 - localDT->Speed_loop_PREV_T;
		Speed_loop_ELAPS_T = 1U;
	}
	//localDT->Speed_loop_PREV_T = rtM->Timing.clockTick1;
	localDT->Speed_loop_RESET_ELAPS_T = false;

	Speederr = rtIN_U_Speedref - rtIN_U_Speedfd;
	if (localDT->Integrator_SYSTEM_ENABLE != 0)
	{
		Integrator = localDT->Integrator_Dstate;
	}
	else if ((rtIN_ResetSignal > 0.0) && (localDT->Integrator_PrevResetState <= 0))
	{
		Integrator = 0.0F;
	}
	else
	{
		Integrator = (real32_T)(0.001 * (real32_T)Speed_loop_ELAPS_T * localDT->Integrator_Prev_U) + localDT->Integrator_Dstate;
	}
	SpeedloopOUT = SpeedKpKi.spd_kp * Speederr + Integrator;

	if (SpeedloopOUT > 1.0F)
	{
		*rtOUT_iq_ref = 1.0F;
	}
	else
	{
		if (SpeedloopOUT < -1.0F)
		{
			*rtOUT_iq_ref = -1.0F;
		}
		else
		{
			*rtOUT_iq_ref = SpeedloopOUT;
		}
		if (SpeedloopOUT >= -1.0F)
		{
			SpeedloopOUT = 0.0F;
		}
		else
		{
			SpeedloopOUT++;
		}
	}

	Speederr *= SpeedKpKi.spd_ki;
	localDT->Integrator_SYSTEM_ENABLE = 0U;
	localDT->Integrator_Dstate = Integrator;
	if (rtIN_ResetSignal > 0.0)
	{
		localDT->Integrator_PrevResetState = 1;
	}
	else if (rtIN_ResetSignal < 0.0)
	{
		localDT->Integrator_PrevResetState = 0;
	}
	else 
	{
		localDT->Integrator_PrevResetState = 2;
	}

	if (SpeedloopOUT > 0.0F)
	{
		tmp = 1;
	}
	else
	{
		tmp = -1;
	}
	if (Speederr > 0.0F)
	{
		tmp_0 = 1;
	}
	else
	{
		tmp_0 = -1;
	}

	if ((SpeedloopOUT != 0.0F) && (tmp == tmp_0))
	{
		localDT->Integrator_Prev_U = 0.0F;
	}
	else
	{
		localDT->Integrator_Prev_U = Speederr;
	}
}
/*函数将输入参数输出为1.0*/
static void SignalMax(real32_T* rtOUT_out1)
{
	*rtOUT_out1 = 1.0F;
}
/*函数将输入参数输出为-1.0*/
static void SignalMin(real32_T* rtOUT_out1)
{
	*rtOUT_out1 = -1.0F;
}
/*Signal函数*/
static void SignalFunction(real32_T rtIN_x, real32_T* rtOUT_y,
	real_T rt_a, real_T rt_c)
{
	*rtOUT_y = 1.0F / (expf((rtIN_x - (real32_T)rt_c) * (real32_T)rt_a) + 1.0F);
}
/*离散滑模观测器*/
/*
rtIN_ialpha：
rtIN_ibeta：
rtIN_ualpha：
rtIN_ubeta：
rtIN_we：

rtOUT_ealpha：
rtOUT_ebeta：
*/
static void DiscretSMO(real32_T rtIN_ialpha, real32_T rtIN_ibeta, real32_T rtIN_ualpha, real32_T rtIN_ubeta, 
	real32_T rtIN_we, real32_T *rtOUT_ealpha, real32_T *rtOUT_ebeta, DT_SMO *localDT)
{
	real32_T eialphaerr;
	real32_T eibetaerr;
	real32_T eialpha;
	real32_T eibeta;
	real32_T Signal_ialphaerr;
	real32_T Signal_ibetaerr;

	*rtOUT_ebeta = localDT->DSMO_eialpha;
	eialpha = (1.0F - Motor.Rs * 5.0E-5F / Motor.L) * *rtOUT_ebeta;
	*rtOUT_ebeta = localDT->DSMO_ealpha;

	Signal_ialphaerr = localDT->DSMO_eialpha;
	eialphaerr = localDT->DSMO_eialpha - rtIN_ialpha;
	SignalFunction(eialphaerr, &Signal_ialphaerr, 5.0, 0.0);
	if (eialphaerr > 0.5)
	{
		SignalMax(&Signal_ialphaerr);
	}
	else if (eialphaerr < -0.5)
	{
		SignalMin(&Signal_ialphaerr);
	}
	else
	{
		Signal_ialphaerr = 2.0F * Signal_ialphaerr - 1.0F;
	}

	eialpha = ((eialpha - 5.0E-5F / Motor.L * *rtOUT_ebeta) +
		5.0E-5F / Motor.L * rtIN_ualpha) - SMO.K * 5.0E-5F * Signal_ialphaerr;

	*rtOUT_ealpha = (*rtOUT_ebeta - 5.0E-5F * rtIN_we * localDT->DSMO_ebeta)
		+ SMO.M * 5.0E-5F * Signal_ialphaerr;

	Signal_ibetaerr = localDT->DSMO_eibeta;
	eibetaerr = localDT->DSMO_eibeta - rtIN_ibeta;
	SignalFunction(eibetaerr, &Signal_ibetaerr, 5.0, 0.0);
	if (eibetaerr > 0.5)
	{
		SignalMax(&Signal_ibetaerr);
	}
	else if (eibetaerr < -0.5)
	{
		SignalMin(&Signal_ibetaerr);
	}
	else
	{
		Signal_ibetaerr = 2.0F * Signal_ibetaerr - 1.0F;
	}
	eibeta = (((1.0F - Motor.Rs * 5.0E-5F / Motor.L) *
		localDT->DSMO_eibeta - 5.0E-5F / Motor.L *
		localDT->DSMO_ebeta) + 5.0E-5F / Motor.L * rtIN_ubeta) -
		SMO.K * 5.0E-5F * Signal_ibetaerr;
	*rtOUT_ebeta = (5.0E-5F * rtIN_we * *rtOUT_ebeta + localDT->DSMO_ebeta)+
		SMO.M * 5.0E-5F * Signal_ibetaerr;

	localDT->DSMO_eialpha = eialpha;
	localDT->DSMO_eibeta = eibeta;
	localDT->DSMO_ealpha = *rtOUT_ealpha;
	localDT->DSMO_ebeta = *rtOUT_ebeta;
}
/*LPF滤波*/
/*
rtIN_we：
*/
static real32_T LPFF(real32_T rtIN_we, DT_LPF* localDT)
{
	real32_T rtOUT_we;
	rtOUT_we = (rtIN_we - localDT->LPF_We) * SMO.LPFfilter +
		localDT->LPF_We;
	localDT->LPF_We = rtOUT_we;
	return rtOUT_we;
}
/*mod函数*/
real32_T rt_modf(real32_T u0, real32_T u1)
{
	real32_T y;
	y = u0;
	if (u1 == 0.0F) {
		if (u0 == 0.0F) {
			y = u1;
		}
	}
	else if (rtIsNaNF(u0) || rtIsNaNF(u1) || rtIsInfF(u0)) {
		y = (rtNaNF);
	}
	else if (u0 == 0.0F) {
		y = 0.0F / u1;
	}
	else if (rtIsInfF(u1)) {
		if ((u1 < 0.0F) != (u0 < 0.0F)) {
			y = u1;
		}
	}
	else {
		boolean_T yEq;
		y = fmodf(u0, u1);
		yEq = (y == 0.0F);
		if ((!yEq) && (u1 > floorf(u1))) {
			real32_T q;
			q = fabsf(u0 / u1);
			yEq = !(fabsf(q - floorf(q + 0.5F)) > FLT_EPSILON * q);
		}

		if (yEq) {
			y = u1 * 0.0F;
		}
		else if ((u0 < 0.0F) != (u1 < 0.0F)) {
			y += u1;
		}
	}
/*
	y = fmod(u0, u1);
	y = (y >= 0 ? y : (y + u1));
*/
	
	return y;
}
/*PLL锁相环*/
/*
rtIN_ealpha：
rtIN_ebeta：

rtOUT_we：
rtOUT_theta：
*/
static void Pll(real32_T rtIN_ealpha, real32_T rtIN_ebeta, real32_T* rtOUT_we,
	real32_T* rtOUT_theta, DT_PLL* localDT)
{
	real32_T ewe;
	real32_T thetaerr;
	real32_T Integral;
	int8_T tmp;
	int8_T tmp_0;

	*rtOUT_theta = localDT->etheta;
	thetaerr = (0.0F - rtIN_ealpha * cosf(localDT->etheta)) -
		rtIN_ebeta * sinf(localDT->etheta);
	ewe = SMO.pll_kp * thetaerr + localDT->Integrator_Dstate;
	if (ewe > 4000.0F)
	{
		*rtOUT_we = 4000.0F;
		ewe -= 4000.0F;
	}
	else
	{
		if (ewe < -4000.0F)
		{
			*rtOUT_we = -4000.0F;
		}
		else
		{
			*rtOUT_we = ewe;
		}
		if (ewe > -4000.0F)
		{
			ewe = 0.0F;
		}
		else
		{
			ewe -= -4000.0F;
		}
	}
	*rtOUT_theta += *rtOUT_we * 5.0E-5F;
	*rtOUT_theta = rt_modf(*rtOUT_theta, 6.28318548F);
	Integral = thetaerr * SMO.pll_ki;
	localDT->etheta = *rtOUT_theta;

	if (ewe > 0.0F)
	{
		tmp = 1;
	}
	else
	{
		tmp = -1;
	}
	if (Integral > 0.0F)
	{
		tmp_0 = 1;
	}
	else
	{
		tmp_0 = -1;
	}
	if ((ewe != 0.0F) && (tmp == tmp_0))
	{
		Integral = 0.0F;
	}
	localDT->Integrator_Dstate += 5.0E-5F * Integral;
}
/*SMO滑膜观测器 + PLL锁相环*/
/*
rtIN_ialpha：
rtIN_ibeta：
rtIN_ualpha：
rtIN_ubeta：

rtOUT_theta：
rtOUT_we：
*/
static void SMO_C(real32_T rtIN_ialpha, real32_T rtIN_ibeta, real32_T rtIN_ualpha, real32_T rtIN_ubeta,
	real32_T* rtOUT_theta, real32_T* rtOUT_we, DT_SMO_C* localDT)
{
	real32_T ealpha;
	real32_T ebeta;
	real32_T welpf;

	DiscretSMO(rtIN_ialpha, rtIN_ibeta, rtIN_ualpha, rtIN_ubeta,
		localDT->ewe, &ealpha, &ebeta, &localDT->DSMO);
	Pll(ealpha, ebeta, rtOUT_we, rtOUT_theta, &localDT->DPLL);
	welpf = LPFF(*rtOUT_we, &localDT->DLPF);
	*rtOUT_we = 1.0F / Motor.Pn * welpf * 9.54929638F;
	localDT->ewe = welpf;
}
/*电流环初始化函数*/
static void Currloop_Init(real_T* rtIN_RsewtSignal, DT_Curr_loop* localDT)
{
	*rtIN_RsewtSignal = 0.0F;
	localDT->Currloop_If3_DiscreteTimeInte_i = 2;
	localDT->Currloop_If3_DiscreteTimeInte_b = 2;
}
/*电流环函数*/
/*
rtIN_ia：
rtIN_ib：
rtIN_ic：
rtIN_iqref：
rtIN_MotorOnOff：
rtIN_Udc：

rtOUT_PWM[3]:
rtOUT_SMOwe:
rtOUT_RsewtSignal:
*/
static void Currloop(real32_T rtIN_ia, real32_T rtIN_ib, real32_T rtIN_ic,
	real32_T rtIN_iqref, real32_T rtIN_MotorOnOff, real32_T rtIN_Udc,
	real32_T rtOUT_PWM[3], real32_T* rtOUT_SMOwe, real32_T* rtOUT_RsewtSignal,
	DT_Curr_loop* localDT)
{
	real32_T ialpha;
	real32_T ibeta;
	real32_T we;
	real32_T IF4_add;
	real32_T IF4_theta;
	real32_T SMOTheta;
	real32_T Currloop_cos;
	real32_T Currloop_sin;
	real32_T idout;
	real32_T udout;
	real32_T iqout;
	real32_T uqout;
	real32_T idIntegral;
	real32_T iqIntegral;
	real32_T OUT_Limit;
	real32_T Switch_Integral_idq;
	real32_T ualpha;
	real32_T ubeta;
	int8_T tmp;
	int8_T tmp_0;

	//clark变换
	ialpha = 0.666666687F * rtIN_ia - (rtIN_ib + rtIN_ic) * 0.333333343F;
	ibeta = (rtIN_ib - rtIN_ic) * 0.577350259F;
	//I/F启动策略
	if (localDT->temporalCounter_i1 < 131071U)
	{
		localDT->temporalCounter_i1++;
	}
	if (localDT->is_active_c3_FOC_Model == 0U)
	{
		localDT->is_active_c3_FOC_Model = 1U;
		localDT->is_c3_FOC_Model = IN_IDLE;
	}
	else
	{
		switch (localDT->is_c3_FOC_Model)
		{
			case IN_AlignStage:
			{
				if (localDT->temporalCounter_i1 >= 1000U)
				{
					localDT->is_c3_FOC_Model = IN_OpenStage;
					localDT->temporalCounter_i1 = 0U;
					localDT->ZReset = 0.0F;
					localDT->cnt = 0.0;
				}
				else if (rtIN_MotorOnOff == 0.0F)
				{
					localDT->is_c3_FOC_Model = IN_IDLE;
				}
				else
				{
					localDT->Motor_state = 2.0F;
					*rtOUT_RsewtSignal = 0.0F;
				}
			}break;
			case IN_IDLE:
			{
				if (rtIN_MotorOnOff == 1.0F)
				{
					localDT->is_c3_FOC_Model = IN_AlignStage;
					localDT->temporalCounter_i1 = 0U;
				}
				else
				{
					localDT->Motor_state = 1.0F;
					*rtOUT_RsewtSignal = 0.0F;
				}
			}break;
			case IN_OpenStage:
			{
				if (localDT->temporalCounter_i1 >= 90000U)
				{
					localDT->is_c3_FOC_Model = IN_ThetaAlign;
					localDT->temporalCounter_i1 = 0U;
				}
				else if (rtIN_MotorOnOff == 0.0F)
				{
					localDT->is_c3_FOC_Model = IN_IDLE;
				}
				else
				{
					if (localDT->cnt == 1.0)
					{
						localDT->ZReset = 1.0F;
					}
					localDT->cnt = 1.0;
					localDT->Motor_state = 3.0F;
					*rtOUT_RsewtSignal = 0.0F;
				}
			}break;
			case IN_RunStage:
			{
				if (rtIN_MotorOnOff == 0.0F)
				{
					localDT->is_c3_FOC_Model = IN_IDLE;
				}
				else
				{
					localDT->Motor_state = 5.0F;
					*rtOUT_RsewtSignal = 1.0F;
				}
			}break;
			default:
			{
				if (localDT->temporalCounter_i1 >= 1000U)
				{
					localDT->is_c3_FOC_Model = IN_RunStage;
				}
				else
				{
					localDT->Motor_state = 4.0F;
					*rtOUT_RsewtSignal = 0.0F;
				}
			}break;
		}
	}

	switch ((int32_T)localDT->Motor_state)
	{
		case 1:
		{
			localDT->Currloop_theta = 0.0F;
			localDT->Currloop_iq = 0.0F;
		}break;
		case 2:
		{
			localDT->Currloop_theta = 0.0F;
			localDT->Currloop_iq = 1.0F;
		}break;
		case 3:
		{
			if ((localDT->ZReset > 0.0F) &&
				(localDT->Currloop_If3_DiscreteTimeInte_i <= 0.0F))
			{
				localDT->Currloop_If3_welimit = 0.0F;
			}
			if (localDT->Currloop_If3_welimit > 3500.0F)
			{
				we = 3500.0F;
			}
			else if (localDT->Currloop_If3_welimit < 0.0F)
			{
				we = 0.0F;
			}
			else
			{
				we = localDT->Currloop_If3_welimit;
			}
			if ((localDT->ZReset > 0.0F) && (localDT->Currloop_If3_DiscreteTimeInte_b <= 0.0F))
			{
				localDT->Currloop_If3_theta = 0.0F;
			}
			localDT->Currloop_theta = rt_modf(localDT->Currloop_If3_theta, 6.28318548F);
			localDT->Currloop_iq = 0.6F;

			localDT->Currloop_If3_welimit += Motor.Pn * 1570.79632F * 0.5F * 5.0E-5F;
			if (localDT->ZReset > 0.0F)
			{
				localDT->Currloop_If3_DiscreteTimeInte_i = 1;
			}
			else if (localDT->ZReset < 0.0F)
			{
				localDT->Currloop_If3_DiscreteTimeInte_i = -1;
			}
			else if (localDT->ZReset == 0.0F)
			{
				localDT->Currloop_If3_DiscreteTimeInte_i = 0;
			}
			else
			{
				localDT->Currloop_If3_DiscreteTimeInte_i = 2;
			}

			localDT->Currloop_If3_theta += 5.0E-5F * we;
			if (localDT->ZReset > 0.0F)
			{
				localDT->Currloop_If3_DiscreteTimeInte_b = 1;
			}
			else if (localDT->ZReset < 0.0F)
			{
				localDT->Currloop_If3_DiscreteTimeInte_b = -1;
			}
			else if (localDT->ZReset == 0.0F)
			{
				localDT->Currloop_If3_DiscreteTimeInte_b = 0;
			}
			else
			{
				localDT->Currloop_If3_DiscreteTimeInte_b = 2;
			}
		}break;
		case 4:
		{
			IF4_theta = rt_modf(Motor.Pn * 3455.7519 * 5.0E-5F + localDT->Currloop_If4_theta,
				6.28318548F);
			localDT->Currloop_theta = IF4_theta;

			IF4_add = localDT->Currloop_If4_UDelay_iq;
			if (localDT->Currloop_If4_UDelay_iq + 0.01F >= 1.0F)
			{
				SignalMax(&IF4_add);
			}
			else
			{
				IF4_add = localDT->Currloop_If4_UDelay_iq + 0.01F;
			}
			localDT->Currloop_iq = 0.6F - IF4_add * 0.2F;
			localDT ->Currloop_If4_theta = IF4_theta;
			localDT->Currloop_If4_UDelay_iq += 0.01F;
		}break;
		case 5:
		{
			SMOTheta = localDT->Currloop_SMO_theta;
			localDT->Currloop_theta = SMOTheta;
			localDT->Currloop_iq = rtIN_iqref;
		}break;
	}
	Currloop_cos = arm_cos_f32(localDT->Currloop_theta);
	Currloop_sin = arm_sin_f32(localDT->Currloop_theta);

	idout = ialpha * Currloop_cos + ibeta * Currloop_sin;
	iqout = ibeta * Currloop_cos - ialpha * Currloop_cos;
	//基于id=0控制策略：id电流环
	idIntegral = (0.0F - idout) * CurrKpKi.curr_ki;
	idout = (0.0F - idout) * CurrKpKi.curr_kp + localDT->Integrator_idAdd;
	if (idout > 10.3923044F)
	{
		OUT_Limit = idout - 10.3923044F;
		udout = 10.3923044F;
		tmp = 1;
	}
	else
	{
		if (idout > -10.3923044F)
		{
			udout = idout;
			OUT_Limit = 0.0F;
		}
		else
		{
			udout = -10.3923044F;
			OUT_Limit = idout - -10.3923044F;
		}
		tmp = -1;
	}
	if (idIntegral > 0.0F)
	{
		tmp_0 = 1;
	}
	else
	{
		tmp_0 = -1;
	}
	if ((OUT_Limit != 0.0F) && (tmp == tmp_0))
	{
		Switch_Integral_idq = 0.0F;
	}
	else
	{
		Switch_Integral_idq = idIntegral;
	}
	localDT->Integrator_idAdd += 5.0E-5F * Switch_Integral_idq;
	//iq电流环
	iqIntegral = (localDT->Currloop_iq - iqout) * CurrKpKi.curr_ki;
	iqout = (localDT->Currloop_iq - iqout) * CurrKpKi.curr_kp + localDT->Integrator_iqAdd;
	if (iqout > 10.3923044F)
	{
		OUT_Limit = iqout - 10.3923044F;
		uqout = 10.3923044F;
		tmp = 1;
	}
	else
	{
		if (iqout > -10.3923044F)
		{
			uqout = iqout;
			OUT_Limit = 0.0F;
		}
		else
		{
			uqout = -10.3923044F;
			OUT_Limit = iqout - -10.3923044F;
		}
		tmp = -1;
	}
	if (iqIntegral > 0.0F)
	{
		tmp_0 = 1;
	}
	else
	{
		tmp_0 = -1;
	}
	if ((OUT_Limit != 0.0F) && (tmp == tmp_0))
	{
		Switch_Integral_idq = 0.0F;
	}
	else
	{
		Switch_Integral_idq = iqIntegral;
	}
	localDT->Integrator_iqAdd += 5.0E-5F * Switch_Integral_idq;
	//Park逆变换
	ualpha = udout * Currloop_cos - uqout * Currloop_sin;
	ubeta = udout * Currloop_sin + uqout * Currloop_cos;
	//SMO+PLL
	SMO_C(ialpha, ibeta, ualpha, ubeta, 
		&localDT->Currloop_SMO_theta, rtOUT_SMOwe, &localDT->DSMO_C);

	//基于均值零序分量注入的SPWM
	OUT_Limit = -0.5F * ualpha;
	Switch_Integral_idq = 0.866025388F * ubeta;

	idIntegral = OUT_Limit + Switch_Integral_idq;//ub
	OUT_Limit -= Switch_Integral_idq;//uc
	iqIntegral = (fminf(fminf(ualpha, idIntegral), OUT_Limit) +
		fmaxf(fmaxf(ualpha, idIntegral), OUT_Limit)) * -0.5F;//u0
	rtOUT_PWM[2] = iqIntegral + ualpha;
	rtOUT_PWM[1] = iqIntegral + idIntegral;
	rtOUT_PWM[0] = iqIntegral + OUT_Limit;
	//三相PWM值
	rtOUT_PWM[0] = (-rtOUT_PWM[0] / rtIN_Udc + 0.5F) * 4000.0F;
	rtOUT_PWM[1] = (-rtOUT_PWM[1] / rtIN_Udc + 0.5F) * 4000.0F;
	rtOUT_PWM[2] = (-rtOUT_PWM[2] / rtIN_Udc + 0.5F) * 4000.0F;
}
/*FOC启动函数1*/
void FOC_Model_step0(void)
{
	real32_T SMOwe;
	(rtM->Timing.RateInteraction.TID0_1)++;
	if ((rtM->Timing.RateInteraction.TID0_1) > 19)
	{
		rtM->Timing.RateInteraction.TID0_1 = 0;
	}
	if (rtOUT_U.speedRef > 20000.0)
	{
		SMO.LPFfilter = 0.0001F;
	}
	else
	{
		SMO.LPFfilter = 0.003F;
	}
	if (rtM->Timing.RateInteraction.TID0_1 == 1)
	{
		FOC_Model_step1();
		rtDT.iqref = rtDT.iqref_speedloop;
	}
	Currloop(rtIN.ia, rtIN.ib, rtIN.ic, rtDT.iqref, rtIN.Motor_OnOff,
		rtIN.Udc, rtDT.SVPWM,&SMOwe, &rtDT.ResetSignal, &rtDT.DCurr_loop);
	rtOUT.PWM1 = rtDT.SVPWM[0];
	rtOUT.PWM2 = rtDT.SVPWM[1];
	rtOUT.PWM3 = rtDT.SVPWM[2];

	if (rtM->Timing.RateInteraction.TID0_1 == 1)
	{
		rtDT.speedfd = SMOwe;
		rtDT.ResetSignal_Buffer = rtDT.ResetSignal;
	}
}
/*FOC启动函数2*/
void FOC_Model_step1(void)
{
	Speedloop(rtM, rtDT.speedfd, rtIN.speedRef,
		rtDT.ResetSignal_Buffer, &rtDT.iqref_speedloop, &rtDT.DSpeed_loop);
	//rtM->Timing.clockTick1++;
}
/*FOC启动函数初始化*/
void FOC_Model_Init(void)
{
	rtIN.Motor_OnOff = 1.0F;
	rtIN.Udc = 18.0F;
	rtIN.speedRef = 20000.0F;
	
	Speedloop_Init(&rtDT.DSpeed_loop);
	Currloop_Init(&rtDT.ResetSignal, &rtDT.DCurr_loop);
	Speedloop_Enable(&rtDT.DSpeed_loop);
}