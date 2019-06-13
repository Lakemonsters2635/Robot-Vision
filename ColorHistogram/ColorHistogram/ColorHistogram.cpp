
// ColorHistogram.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "ColorHistogram.h"
#include "ColorHistogramDlg.h"

#include "example.hpp"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

void
RgbToHsv(unsigned short R, unsigned short G, unsigned short B, unsigned short& H, unsigned short& S, unsigned short& V)
{
	unsigned short rgbMin, rgbMax;

	rgbMin = R < G ? (R < B ? R : B) : (G < B ? G : B);
	rgbMax = R > G ? (R > B ? R : B) : (G > B ? G : B);

	V = rgbMax;
	if (V == 0)
	{
		H = 0;
		S = 0;
		return;
	}

	S = unsigned short(0.5 + 255.0 * long(rgbMax - rgbMin) / V);
	if (S == 0)
	{
		H = 0;
		return;
	}

	if (rgbMax == R)
		H = unsigned short(0.5 + 43.0 * (G - B) / (rgbMax - rgbMin));
	else if (rgbMax == G)
		H = unsigned short(85.5 + 43.0 * (B - R) / (rgbMax - rgbMin));
	else
		H = unsigned short(171.5 + 43.0 * (R - G) / (rgbMax - rgbMin));

}

// CColorHistogramApp

BEGIN_MESSAGE_MAP(CColorHistogramApp, CWinApp)
	ON_COMMAND(ID_HELP, &CWinApp::OnHelp)
END_MESSAGE_MAP()


// CColorHistogramApp construction

CColorHistogramApp::CColorHistogramApp()
{
	// support Restart Manager
	m_dwRestartManagerSupportFlags = AFX_RESTART_MANAGER_SUPPORT_RESTART;

	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}


// The one and only CColorHistogramApp object

CColorHistogramApp theApp;


// CColorHistogramApp initialization

BOOL CColorHistogramApp::InitInstance()
{
	// InitCommonControlsEx() is required on Windows XP if an application
	// manifest specifies use of ComCtl32.dll version 6 or later to enable
	// visual styles.  Otherwise, any window creation will fail.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// Set this to include all the common control classes you want to use
	// in your application.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();


	AfxEnableControlContainer();

	// Create the shell manager, in case the dialog contains
	// any shell tree view or shell list view controls.
	CShellManager *pShellManager = new CShellManager;

	// Activate "Windows Native" visual manager for enabling themes in MFC controls
	CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows));

	// Standard initialization
	// If you are not using these features and wish to reduce the size
	// of your final executable, you should remove from the following
	// the specific initialization routines you do not need
	// Change the registry key under which our settings are stored
	// TODO: You should modify this string to be something appropriate
	// such as the name of your company or organization
	SetRegistryKey(_T("Local AppWizard-Generated Applications"));
	
	for (;;)
	{
		CColorHistogramDlg dlg;
		m_pMainWnd = &dlg;

		window xyz(1280, 720, "Camera Data");

		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		rs2::points points;

		// Declare depth colorizer for pretty visualization of depth data
		rs2::colorizer color_map;
		// Declare rates printer for showing streaming rates of the enabled streams.
		rs2::rates_printer printer;

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;
		// Start streaming with default recommended configuration
		pipe.start();

		rs2::frameset frames;

		// Wait for the next set of frames from the camera
		while (xyz)
		{
			frames = pipe.wait_for_frames();
			rs2::frameset data = frames.apply_filter(printer).apply_filter(color_map);
			xyz.show(data);
		}

		auto color = frames.get_color_frame();
		auto width = color.get_width();
		auto height = color.get_height();
		auto BPP = color.get_bytes_per_pixel();
		auto bPP = color.get_bits_per_pixel();
		auto stride = color.get_stride_in_bytes();
		uint8_t* pixels = (uint8_t*)color.get_data();

		dlg.m_pHPixels = new BYTE[width * height];			// Cache the H values so we don't need to recalculate

		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				int bytes = x * BPP;
				int strides = y * stride;
				int Index = (bytes + strides);

				unsigned short R = pixels[Index];
				unsigned short G = pixels[Index + 1];
				unsigned short B = pixels[Index + 2];

				dlg.R[R]++;
				dlg.G[G]++;
				dlg.B[B]++;

				unsigned short H;
				unsigned short S;
				unsigned short V;

				RgbToHsv(R, G, B, H, S, V);
				dlg.H[H & 0xff]++;
				dlg.S[S & 0xff]++;
				dlg.V[V & 0xff]++;

				dlg.m_pHPixels[x + y*width] = H & 0xff;
			}
		}

		dlg.m_pFrame = &color;

		INT_PTR nResponse = dlg.DoModal();
		if (nResponse == IDOK)
		{
			continue;
		}
		else if (nResponse == IDCANCEL)
		{
			break;
		}
		else if (nResponse == -1)
		{
			TRACE(traceAppMsg, 0, "Warning: dialog creation failed, so application is terminating unexpectedly.\n");
			TRACE(traceAppMsg, 0, "Warning: if you are using MFC controls on the dialog, you cannot #define _AFX_NO_MFC_CONTROLS_IN_DIALOGS.\n");
			break;
		}
	}

	// Delete the shell manager created above.
	if (pShellManager != nullptr)
	{
		delete pShellManager;
	}

#if !defined(_AFXDLL) && !defined(_AFX_NO_MFC_CONTROLS_IN_DIALOGS)
	ControlBarCleanUp();
#endif

	// Since the dialog has been closed, return FALSE so that we exit the
	//  application, rather than start the application's message pump.
	return FALSE;
}

