#pragma once

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
	DLL_PREFIX void Interface_1(void);
}
#endif

