
// ColorHistogramDlg.h : header file
//

#pragma once
#include "CHistogram.h"
#include "CPicture.h"

// CColorHistogramDlg dialog
class CColorHistogramDlg : public CDialogEx
{
// Construction
public:
	CColorHistogramDlg(CWnd* pParent = nullptr);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_COLORHISTOGRAM_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

	CSliderCtrl m_LeftSlider;
	CSliderCtrl m_RightSlider;
	CString m_strLeftSlider;
	CString m_strRightSlider;
	int m_nLeftSlider;
	int m_nRightSlider;

public:
	CHistogram m_H;
	CHistogram m_S;
	CHistogram m_V;
	CHistogram m_R;
	CHistogram m_G;
	CHistogram m_B;

	long R[256];
	long G[256];
	long B[256];
	long H[256];
	long S[256];
	long V[256];

	CString m_strHLabel;
	CString m_strSLabel;
	CString m_strVLabel;
	CString m_strRLabel;
	CString m_strGLabel;
	CString m_strBLabel;

	rs2::video_frame*	m_pFrame;
	int					m_VideoWidth;
	int					m_VideoHeight;
	int					m_ImageDlgWidth;
	int					m_ImageDlgHeight;
	CImage				m_ImageImage;
	CBitmap				m_ImageBitmap;
	CPicture			m_Image;
	BOOL BuildBitmapFromFrame();

	BYTE*				m_pHPixels;

	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
};
