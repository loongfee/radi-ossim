// This is the main DLL file.
#include <windows.h>
#include "stdafx.h"

#include "ossim_core_project.h"

extern "C"
BOOL WINAPI DllMain(HINSTANCE hInstance,  /* handle to DLL module */
                    DWORD     dwReason,   /* reason for calling function */
		    LPVOID    lpReserved )/* reserved */
{
   /* Nothing to do here yet. */
   return TRUE;
}

