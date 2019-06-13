// CPicture.cpp : implementation file
//

#include "stdafx.h"
#include "ColorHistogram.h"
#include "CPicture.h"


// CPicture

IMPLEMENT_DYNAMIC(CPicture, CWnd)

CPicture::CPicture()
{
	RegisterWindowClass();

}

CPicture::~CPicture()
{
}


BEGIN_MESSAGE_MAP(CPicture, CWnd)
	ON_WM_ERASEBKGND()
	ON_WM_PAINT()
END_MESSAGE_MAP()



// CPicture message handlers

BOOL CPicture::RegisterWindowClass()
{
	WNDCLASS wndcls;
	HINSTANCE hInst = AfxGetInstanceHandle();

	if (!(::GetClassInfo(hInst, PICTURE_CLASSNAME, &wndcls)))
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
		wndcls.lpszClassName = PICTURE_CLASSNAME;

		if (!AfxRegisterClass(&wndcls))
		{
			AfxThrowResourceException();
			return FALSE;
		}
	}

	return TRUE;
}





BOOL CPicture::OnEraseBkgnd(CDC* pDC)
{
	return CWnd::OnEraseBkgnd(pDC);
}


void CPicture::OnPaint()
{
	// Draw the bitmap - if we have one.
	if (m_pBitmap->GetSafeHandle() != NULL)
	{
		CPaintDC dc(this); // device context for painting

		// Create memory DC
		CDC MemDC;
		if (!MemDC.CreateCompatibleDC(&dc))
			return;

		// Get Size of Display area
		CRect rect;
		GetClientRect(rect);

		// Get size of bitmap
		BITMAP bm;
		m_pBitmap->GetBitmap(&bm);

		// Draw the bitmap
		CBitmap* pOldBitmap = (CBitmap*)MemDC.SelectObject(m_pBitmap);
		dc.StretchBlt(0, 0, rect.Width(), rect.Height(),
			&MemDC,
			0, 0, bm.bmWidth, bm.bmHeight,
			SRCCOPY);
		MemDC.SelectObject(pOldBitmap);
	}

	// Do not call CWnd::OnPaint() for painting messages
}

