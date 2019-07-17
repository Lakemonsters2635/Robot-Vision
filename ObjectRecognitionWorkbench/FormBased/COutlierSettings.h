#pragma once


// COutlierSettings dialog

class COutlierSettings : public CDialogEx
{
	DECLARE_DYNAMIC(COutlierSettings)

public:
	COutlierSettings(long lMeanK, double dStdDevMultiplier, CWnd* pParent = nullptr);   // standard constructor
	virtual ~COutlierSettings();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_OUTLIER_SETTINGS };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	long m_lMeanK;
	double m_dStdDevMultiplier;
};
