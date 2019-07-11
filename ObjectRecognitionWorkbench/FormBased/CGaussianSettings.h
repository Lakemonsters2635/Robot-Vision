#pragma once


// CGaussianSettings dialog

class CGaussianSettings : public CDialogEx
{
	DECLARE_DYNAMIC(CGaussianSettings)

public:
	CGaussianSettings(CSize size, double dSigmaX, double dSigmaY, int nBorderType, CWnd* pParent = nullptr);   // standard constructor
	virtual ~CGaussianSettings();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GAUSSIAN_SETTINGS };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	int m_nSizeX;
	int m_nSizeY;
	double m_dSigmaX;
	double m_dSigmaY;
	int m_nBorderType;
	virtual BOOL OnInitDialog();
	CString m_strBorderType;
	afx_msg void OnCbnSelchangeBorderType();
	CComboBox m_BorderType;
};
