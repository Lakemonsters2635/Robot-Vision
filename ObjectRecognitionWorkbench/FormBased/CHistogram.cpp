// CHistogram.cpp : implementation file
//

#include "stdafx.h"
#include "CHistogram.h"


// CHistogram

IMPLEMENT_DYNAMIC(CHistogram, CWnd)


CHistogram::~CHistogram()
{
}


BEGIN_MESSAGE_MAP(CHistogram, CWnd)
	ON_WM_ERASEBKGND()
	ON_WM_PAINT()
	ON_WM_DRAWITEM()
END_MESSAGE_MAP()



// CHistogram message handlers




BOOL CHistogram::OnEraseBkgnd(CDC* pDC)
{
	return CWnd::OnEraseBkgnd(pDC);
}


void CHistogram::OnPaint()
{
	CPaintDC dc(this); // device context for painting
					   // TODO: Add your message handler code here
					   // Do not call CWnd::OnPaint() for painting messages

	CRect rectClient;
	GetClientRect(&rectClient);

	//CRect rectWindow;
	//GetWindowRect(&rectWindow);

	CBrush brushBlack;
//	brushBlack.CreateSolidBrush(RGB(255,255,255));
	brushBlack.CreateSolidBrush(RGB(0, 0, 0));
	dc.FillRect(&rectClient, &brushBlack);

	CPen pen;
	pen.CreatePen(PS_SOLID, 1, RGB(255, 0, 0));
	auto oldPen = dc.SelectObject(&pen);

	//dc.MoveTo(rectClient.BottomRight());
	//dc.LineTo(rectClient.TopLeft());

	auto maxValue = m_Values[0];

	for (int i = 1; i < sizeof(m_Values)/sizeof(m_Values[0]); i++)
	{
		maxValue = max(maxValue, m_Values[i]);
	}

	if (maxValue > 0)
	{
		for (int x = 0; x < rectClient.Width(); x++)
		{
			auto i = x * 256 / rectClient.Width();
			long y = m_Values[i] * rectClient.Height() / maxValue;

			dc.MoveTo(x + rectClient.left, rectClient.bottom);
			dc.LineTo(x + rectClient.left, rectClient.bottom - y);
		}
	}

	if (m_nLeftSlider != -1 || m_nRightSlider != -1)
	{
		CPen penLines;
		penLines.CreatePen(PS_SOLID, 1, RGB(0, 255, 0));
		dc.SelectObject(&penLines);

		if (m_nLeftSlider != -1)
		{
			dc.MoveTo(m_nLeftSlider * rectClient.Width() / 255 + rectClient.left, rectClient.bottom);
			dc.LineTo(m_nLeftSlider * rectClient.Width() / 255 + rectClient.left, rectClient.top);
		}

		if (m_nRightSlider != -1)
		{
			dc.MoveTo(m_nRightSlider * rectClient.Width() / 255 + rectClient.left, rectClient.bottom);
			dc.LineTo(m_nRightSlider * rectClient.Width() / 255 + rectClient.left, rectClient.top);
		}
	}

	dc.SelectObject(&oldPen);
}

BOOL CHistogram::RegisterWindowClass()
{
	WNDCLASS wndcls;
	HINSTANCE hInst = AfxGetInstanceHandle();

	if (!(::GetClassInfo(hInst, HISTOGRAM_CLASSNAME, &wndcls)))
	{
		// otherwise we need to register a new class
		wndcls.style = CS_DBLCLKS | CS_HREDRAW | CS_VREDRAW;
		wndcls.lpfnWndProc = ::DefWindowProc;
		wndcls.cbClsExtra = wndcls.cbWndExtra = 0;
		wndcls.hInstance = hInst;
		wndcls.hIcon = NULL;
		wndcls.hCursor = AfxGetApp()->LoadStandardCursor(IDC_ARROW);
		wndcls.hbrBackground = (HBRUSH)(COLOR_3DFACE + 1);
		wndcls.lpszMenuName = NULL;
		wndcls.lpszClassName = HISTOGRAM_CLASSNAME;

		if (!AfxRegisterClass(&wndcls))
		{
			AfxThrowResourceException();
			return FALSE;
		}
	}

	return TRUE;
}


bool CHistogram::SetLeftSlider(int nValue)
{
	m_nLeftSlider = nValue;
	Invalidate(true);
	return false;
}

bool CHistogram::SetRightSlider(int nValue)
{
	m_nRightSlider = nValue;
	Invalidate();
	return false;
}


void CHistogram::OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct)
{
	// TODO: Add your message handler code here and/or call default

	CWnd::OnDrawItem(nIDCtl, lpDrawItemStruct);
}
