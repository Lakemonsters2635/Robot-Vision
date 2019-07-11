#pragma once


// CCannySettings dialog

class CCannySettings : public CDialogEx
{
	DECLARE_DYNAMIC(CCannySettings)

public:
	CCannySettings(double dThreshhold1, double dThreshhold2, int nAperture, bool bL2Gradient, CWnd* pParent = nullptr);   // standard constructor
	virtual ~CCannySettings();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CANNY_SETTINGS };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	double m_dThreshhold1;
	double m_dThreshhold2;
	int m_nAperture;
	BOOL m_bL2Gradient;
};
