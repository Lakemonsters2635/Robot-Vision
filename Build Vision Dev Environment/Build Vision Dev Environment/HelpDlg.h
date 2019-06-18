#pragma once


// CHelpDlg dialog

class CHelpDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CHelpDlg)

public:
	CHelpDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CHelpDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDC_MYHELP };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
};
