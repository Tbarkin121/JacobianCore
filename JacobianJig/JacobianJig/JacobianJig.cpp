// JacobianJig.cpp : Defines the entry point for the application.
//

#include "JacobianJig.h"
#include <Windows.h>
#include <stdio.h>
#include <iostream>

using namespace std;
typedef void (*fun1)(void);

int main()
{
	cout << "Hello CMake." << endl;

	HINSTANCE hinstLib;
	BOOL fFreeResult, fRunTimeLinkSuccess = FALSE;

	// Get a handle to the DLL module.
	BOOL set_dir_res = SetDllDirectory(TEXT("C:\\Users\\Plutonium\\MachineLearning\\git\\JacobianCore\\JacobianCore\\out\\build\\x64-Release\\JacobianCore"));
	hinstLib = LoadLibrary(TEXT("JacobianCore.dll"));

	cout << "DLL Linked" << endl;
	cout << "Lib Address : " << hinstLib << endl;

	if (hinstLib != NULL)
	{
		fun1 I1 = (fun1)GetProcAddress(hinstLib, "Interface_1");
		
		cout <<  (I1) << endl;
		if (NULL != I1)
		{
			fRunTimeLinkSuccess = TRUE;
			cout << "Interface_1 Function Found" << endl;
			I1();
		}
	}

	fFreeResult = FreeLibrary(hinstLib);

	// If unable to call the DLL function, use an alternative.
	if (!fRunTimeLinkSuccess)
	{
		printf("Message printed from executable\n");
		cout << "DLL Freed" << endl;
	}
	return 0;
}
