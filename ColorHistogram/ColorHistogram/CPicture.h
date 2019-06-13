#pragma once

#define	PICTURE_CLASSNAME		_T("MFCPicture")


// CPicture

class CPicture : public CWnd
{
	DECLARE_DYNAMIC(CPicture)

public:
	CPicture();
	virtual ~CPicture();
	void SetBitmap(CBitmap* pBitmap) { m_pBitmap = pBitmap; }

protected:
	BOOL RegisterWindowClass();

	CBitmap* m_pBitmap;

	DECLARE_MESSAGE_MAP()
public:
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnPaint();
};


