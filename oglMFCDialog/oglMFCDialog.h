// oglMFCDialog.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols


// CoglMFCDialogApp:
// See oglMFCDialog.cpp for the implementation of this class
//

class CoglMFCDialogApp : public CWinApp
{
public:
	CoglMFCDialogApp();

// Overrides
	public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CoglMFCDialogApp theApp;