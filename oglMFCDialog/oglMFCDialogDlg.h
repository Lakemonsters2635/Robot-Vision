// oglMFCDialogDlg.h : header file
//
#pragma once

#include "OpenGLControl.h"

class CoglMFCDialogDlg : public CDialog
{
	private:
		COpenGLControl m_oglWindow;

	// Construction
	public:
		CoglMFCDialogDlg(CWnd* pParent = NULL);	// standard constructor

		// Dialog Data
		enum { IDD = IDD_OGLMFCDIALOG_DIALOG };

	protected:
		virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

	// Implementation
	protected:
		HICON m_hIcon;

		// Generated message map functions
		virtual BOOL OnInitDialog();
		afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
		afx_msg void OnPaint();
		afx_msg void OnSize(UINT nType, int cx, int cy);
		afx_msg HCURSOR OnQueryDragIcon();
		DECLARE_MESSAGE_MAP()
};