// TandemAeroJig.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <Windows.h>
#include <stdio.h>
#include <iostream>
#include <vector>
using namespace std;

typedef void (*fun1)(void);

int main()
{
    std::cout << "Hello DLL!\n";


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

        if (NULL != I1)
        {
            fRunTimeLinkSuccess = TRUE;
            cout << "Interface_1 Function Found" << endl;
            I1();                 
        }
        
        // Free the DLL module.

        fFreeResult = FreeLibrary(hinstLib);

        // If unable to call the DLL function, use an alternative.
        if (!fRunTimeLinkSuccess)
            printf("Message printed from executable\n");
        cout << "DLL Freed" << endl;

        return 0;
    }

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
