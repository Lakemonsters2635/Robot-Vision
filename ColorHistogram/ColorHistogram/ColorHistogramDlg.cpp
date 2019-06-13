
// ColorHistogramDlg.cpp : implementation file
//

#include "stdafx.h"
#include "resource.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "ColorHistogram.h"
#include "ColorHistogramDlg.h"
#include "afxdialogex.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CString
AverageAndSD(long values[256], bool bIsH)		// bIsH means we are computing the H channel which can wrap.
{
	int iMin = bIsH ? 128 : 0;
	int iMax = bIsH ? 384 : 256;

	long nPoints = 0;
	double dSum = 0;
	double dAverage, dSDSum = 0;

	for (int i = iMin; i < iMax; i++)
	{
		nPoints += values[i%256];
		dSum += i * values[i % 256];
	}

	dAverage = dSum / nPoints;

	for (int i = iMin; i < iMax; i++)
	{
		double dDiff = (i - dAverage);
		dSDSum += values[i % 256] * dDiff * dDiff;
	}

	CString strRes;
	strRes.Format(_T("%.0f ± %.0f"), dAverage - 2*iMin, sqrt(dSDSum / nPoints));

	if (bIsH)
		strRes = strRes + _T("  ") + AverageAndSD(values, 0);
	
	return strRes;
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CColorHistogramDlg dialog



CColorHistogramDlg::CColorHistogramDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_COLORHISTOGRAM_DIALOG, pParent)
	, m_strHLabel(_T(""))
	, m_strSLabel(_T(""))
	, m_strVLabel(_T(""))
	, m_strRLabel(_T(""))
	, m_strGLabel(_T(""))
	, m_strBLabel(_T(""))
	, m_strLeftSlider(_T(""))
	, m_strRightSlider(_T(""))
	, m_nLeftSlider(0)
	, m_nRightSlider(40)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	ZeroMemory(R, sizeof(R));
	ZeroMemory(G, sizeof(G));
	ZeroMemory(B, sizeof(B));
	ZeroMemory(H, sizeof(H));
	ZeroMemory(S, sizeof(S));
	ZeroMemory(V, sizeof(V));
}

void CColorHistogramDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_H, m_H);
	DDX_Control(pDX, IDC_S, m_S);
	DDX_Control(pDX, IDC_V, m_V);
	DDX_Control(pDX, IDC_R, m_R);
	DDX_Control(pDX, IDC_G, m_G);
	DDX_Control(pDX, IDC_B, m_B);
	DDX_Text(pDX, IDC_H_LABEL, m_strHLabel);
	DDX_Text(pDX, IDC_S_LABEL, m_strSLabel);
	DDX_Text(pDX, IDC_V_LABEL, m_strVLabel);
	DDX_Text(pDX, IDC_R_LABEL, m_strRLabel);
	DDX_Text(pDX, IDC_G_LABEL, m_strGLabel);
	DDX_Text(pDX, IDC_B_LABEL, m_strBLabel);
	DDX_Control(pDX, IDC_IMAGE, m_Image);
	DDX_Control(pDX, IDC_LEFT_SLIDER, m_LeftSlider);
	DDX_Control(pDX, IDC_RIGHT_SLIDER, m_RightSlider);
	DDX_Text(pDX, IDC_LEFT_VALUE, m_strLeftSlider);
	DDX_Text(pDX, IDC_RIGHT_VALUE, m_strRightSlider);
}

BEGIN_MESSAGE_MAP(CColorHistogramDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
//	ON_STN_CLICKED(IDC_H_LABEL, &CColorHistogramDlg::OnStnClickedHLabel)
ON_WM_HSCROLL()
END_MESSAGE_MAP()


// CColorHistogramDlg message handlers

BOOL CColorHistogramDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

#define INIT_HIST(c, b)			\
	m_##c.SetData(c);			\
	m_##c.Invalidate(TRUE);		\
	m_str##c##Label = AverageAndSD(c, b)

	INIT_HIST(H, 1);
	INIT_HIST(S, 0);
	INIT_HIST(V, 0);
	INIT_HIST(R, 0);
	INIT_HIST(G, 0);
	INIT_HIST(B, 0);

	UpdateData(FALSE);

	CRect rectImage;
	m_Image.GetClientRect(&rectImage);
	m_ImageDlgWidth = rectImage.Width();
	m_ImageDlgHeight = rectImage.Height();

//	m_Image.ModifyStyle(SS_BLACKFRAME, SS_BITMAP);

	//m_ImageImage.Load(_T("file.bmp"));

	// Create the full-size image that will receive the frame bits
	m_ImageImage.Create(m_pFrame->get_width(), m_pFrame->get_height(), m_pFrame->get_bits_per_pixel());
	
	BuildBitmapFromFrame();

	m_Image.SetBitmap(&m_ImageBitmap);

	m_LeftSlider.SetRange(0, 255);
	m_LeftSlider.SetPos(0);
	m_RightSlider.SetRange(0, 255);
	m_RightSlider.SetPos(255);

	UpdateData(FALSE);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CColorHistogramDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CColorHistogramDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CColorHistogramDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}





BOOL CColorHistogramDlg::BuildBitmapFromFrame()
{
	auto width = m_pFrame->get_width();
	auto height = m_pFrame->get_height();
	auto BPP = m_pFrame->get_bytes_per_pixel();
	auto stride = m_pFrame->get_stride_in_bytes();
	uint8_t* pixels = (uint8_t*) m_pFrame->get_data();

//	auto cBytes = width * height * BPP;

#if 0
	// Copy the video pixels into m_ImageImage

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
			m_ImageImage.SetPixelRGB(x, y, R, G, B);
		}
	}
#else
	auto nPitch = m_ImageImage.GetPitch();
	BYTE* pDest = (BYTE*) m_ImageImage.GetBits() + (height-1)*nPitch;

	BYTE* pSource = pixels + (height - 1)*stride;

	for (int row = 0; row < height; row++)
	{
		BYTE* pDR = pDest;
		BYTE* pDG = pDest + 1;
		BYTE* pDB = pDest + 2;

		BYTE* pSB = pSource;
		BYTE* pSG = pSource + 1;
		BYTE* pSR = pSource + 2;

		for (int col = 0; col < width; col++)
		{
			int H = m_pHPixels[(height-(row+1)) * width + col];
			bool bCopy = true;

			if (m_nLeftSlider < m_nRightSlider)
			{
				if (H < m_nLeftSlider || H > m_nRightSlider)
				{
					*pDR = 0;
					*pDG = 0;
					*pDB = 0;
					bCopy = false;
				}
			}
			else
			{
				if (H < m_nLeftSlider && H > m_nRightSlider)
				{
					*pDR = 0;
					*pDG = 0;
					*pDB = 0;
					bCopy = false;
				}
			}
			if (bCopy)
			{
				*pDR = *pSR;
				*pDG = *pSG;
				*pDB = *pSB;
			}

			pDR += 3;
			pDG += 3;
			pDB += 3;
			pSR += 3;
			pSG += 3;
			pSB += 3;
		}
		//memcpy(pDest, pSource, BPP*width);
		pDest = (BYTE*) pDest - nPitch;
		pSource = (BYTE*) pSource - stride;
	}
#endif

	// Now shrink the image to fit the dialog control

	CImage sm_img;
	sm_img.Create(m_ImageDlgWidth, m_ImageDlgHeight, 8*BPP);

	HDC sm_hdc = sm_img.GetDC();
	m_ImageImage.StretchBlt(sm_hdc, 0, 0, m_ImageDlgWidth, m_ImageDlgHeight, SRCCOPY);
	sm_img.ReleaseDC();

	// Now attach the reduced image bits to the bitmap

	if (m_ImageBitmap.m_hObject != NULL)
		m_ImageBitmap.Detach();

	m_ImageBitmap.Attach(sm_img.Detach());

	return 0;
}


void CColorHistogramDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	int value = ((CSliderCtrl*) pScrollBar)->GetPos();

	switch (pScrollBar->GetDlgCtrlID())
	{
	case IDC_LEFT_SLIDER:
		m_H.SetLeftSlider(value);
		m_strLeftSlider.Format(_T("%3d"), value);
		m_nLeftSlider = value;
		break;

	case IDC_RIGHT_SLIDER:
		m_H.SetRightSlider(value);
		m_strRightSlider.Format(_T("%3d"), value);
		m_nRightSlider = value;
		break;

	}

	UpdateData(FALSE);
	BuildBitmapFromFrame();
	m_Image.Invalidate(TRUE);

	CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}
