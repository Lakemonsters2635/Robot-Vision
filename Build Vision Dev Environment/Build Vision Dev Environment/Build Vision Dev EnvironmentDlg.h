
// Build Vision Dev EnvironmentDlg.h : header file
//

#pragma once


// CBuildVisionDevEnvironmentDlg dialog
class CBuildVisionDevEnvironmentDlg : public CDialogEx
{
// Construction
public:
	CBuildVisionDevEnvironmentDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_BUILDVISIONDEVENVIRONMENT_DIALOG };
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

public:
	CString m_strTargetDirectory;
	CString m_strRealSenseDirectory;
	CString m_strPCLDirectory;
	CString m_strLZ4Directory;
	CString m_strOpenCVDirectory;
	CString m_strPCLIncludeDirectory;
	CString m_strVTKIncludeDirectory;
	CString m_strBoostIncludeDirectory;

//	BOOL BuildDirectoryTree();

	afx_msg void OnBnClickedBrowseTarget();
	afx_msg void OnBnClickedBrowseRealsense();
	afx_msg void OnBnClickedGo();
	afx_msg void OnBnClickedHelp();
	BOOL m_bAddRealSense;
	BOOL m_bAddPCL;
	BOOL m_bAddOpenCV;
	BOOL m_bAddLZ4;
	afx_msg void OnEnChangeTargetDirectory();
};
