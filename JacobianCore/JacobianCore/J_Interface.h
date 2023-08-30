#pragma once

#include <stdint.h>

#define EXPORT_LIB
#ifdef EXPORT_LIB
#define DLL_PREFIX __declspec(dllexport)
#else
#define DLL_PREFIX __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C"
{

	//DLL_PREFIX void Init(void);
	DLL_PREFIX void I_Init(uint16_t n_seg);
	DLL_PREFIX void I_SetTarget(float x_targ, float y_targ);
	DLL_PREFIX void I_Control_Update(void);
	DLL_PREFIX void I_Get_Pos(float x_pos[], float y_pos[]);

}
#endif

