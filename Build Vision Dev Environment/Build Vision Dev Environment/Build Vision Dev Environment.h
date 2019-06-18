
// Build Vision Dev Environment.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CBuildVisionDevEnvironmentApp:
// See Build Vision Dev Environment.cpp for the implementation of this class
//

class CBuildVisionDevEnvironmentApp : public CWinApp
{
public:
	CBuildVisionDevEnvironmentApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CBuildVisionDevEnvironmentApp theApp;