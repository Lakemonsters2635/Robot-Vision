#pragma once


// CHistogram

#define	HISTOGRAM_CLASSNAME		_T("MFCHistogramCtrl")

class CHistogram : public CWnd
{
	DECLARE_DYNAMIC(CHistogram)

	typedef long ComponentValues[256];

public:
	CHistogram()
		: m_nLeftSlider(-1)
		, m_nRightSlider(-1)
	{
		ZeroMemory(m_Values, sizeof(m_Values));
		RegisterWindowClass();
	}

	virtual ~CHistogram();

	void SetData(ComponentValues values)
	{
		memcpy_s(m_Values, sizeof(m_Values), values, 1024);
	}

	BOOL Create(CWnd* pParentWnd, const RECT& rect, UINT nID, DWORD dwStyle = WS_VISIBLE)
	{
		return CWnd::Create(HISTOGRAM_CLASSNAME, _T(""), dwStyle, rect, pParentWnd, nID);
	}

protected:
	ComponentValues	m_Values;
	int				m_nLeftSlider;
	int				m_nRightSlider;

	BOOL RegisterWindowClass();

	DECLARE_MESSAGE_MAP()
public:
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnPaint();
	bool SetLeftSlider(int nValue);
	bool SetRightSlider(int nValue);
	afx_msg void OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct);
};


