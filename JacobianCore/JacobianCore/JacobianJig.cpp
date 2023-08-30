// TandemAeroJig.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <Windows.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <chrono>

using namespace std;
using namespace std::chrono;


typedef void (*fun1)(uint16_t);                 //void I_Init(uint16_t n_seg);
typedef void (*fun2)(float, float);             //void I_SetTarget(float x_targ, float y_targ);
typedef void (*fun3)(void);                     //void I_Control_Update(void);
typedef void (*fun4)(float*, float*);           //void I_Get_Pos(float x_pos[], float y_pos[]);

const uint16_t number_of_segments = 5;
std::chrono::time_point<std::chrono::system_clock> tstart, tend;
std::chrono::duration<double> elapsed_seconds;

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

        fun1 I_Init = (fun1)GetProcAddress(hinstLib, "I_Init");
        fun2 I_SetTarget = (fun2)GetProcAddress(hinstLib, "I_SetTarget");
        fun3 I_Control_Update = (fun3)GetProcAddress(hinstLib, "I_Control_Update");
        fun4 I_Get_Pos = (fun4)GetProcAddress(hinstLib, "I_Get_Pos");

        if (NULL != I_Init)
        {
            fRunTimeLinkSuccess = TRUE;
            cout << "I_Init Function Found" << endl;
            I_Init(number_of_segments);
        }
        if (NULL != I_SetTarget)
        {
            fRunTimeLinkSuccess = TRUE;
            cout << "I_SetTarget Function Found" << endl;
            I_SetTarget(-0.33, 0.44);
        }
        if ( (NULL != I_Control_Update) && (NULL != I_Get_Pos) )
        {

            fRunTimeLinkSuccess = TRUE;
            cout << "Starting Control Loop" << endl;
            float x_pos_arr[number_of_segments];
            float y_pos_arr[number_of_segments];
            for (uint16_t i = 0; i < 100; i++)
            {
                tstart = std::chrono::system_clock::now();
                I_Control_Update();
                tend = std::chrono::system_clock::now();
                elapsed_seconds = tend - tstart;
                cout << "Update Time : " << (float)elapsed_seconds.count() << endl;

                I_Get_Pos(x_pos_arr, y_pos_arr);
                cout << " End Effector Pos (x,y) :" << x_pos_arr[number_of_segments] << ", " << y_pos_arr[number_of_segments] << ")" << endl;
                //cout << " End Effector Pos (x,y) :" << x_pos_arr << ", " << y_pos_arr << ")" << endl;
            }
            cout << "Update Time2 : " << (float)elapsed_seconds.count() << endl;

            
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
