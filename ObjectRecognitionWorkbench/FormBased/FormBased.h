
// FormBased.h : main header file for the FormBased application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CFormBasedApp:
// See FormBased.cpp for the implementation of this class
//

class CFormBasedApp : public CWinApp
{
public:
	CFormBasedApp() noexcept;


// Overrides
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
	virtual BOOL OnIdle(LONG lCount);
};

extern CFormBasedApp theApp;
