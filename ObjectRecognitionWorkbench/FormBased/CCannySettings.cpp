// CCannySettings.cpp : implementation file
//

#include "stdafx.h"
#include "FormBased.h"
#include "CCannySettings.h"
#include "afxdialogex.h"


// CCannySettings dialog

IMPLEMENT_DYNAMIC(CCannySettings, CDialogEx)

CCannySettings::CCannySettings(double dThreshhold1, double dThreshhold2, int nAperture, bool bL2Gradient, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_CANNY_SETTINGS, pParent)
	, m_dThreshhold1(dThreshhold1)
	, m_dThreshhold2(dThreshhold2)
	, m_nAperture(nAperture)
	, m_bL2Gradient(bL2Gradient)
{

}

CCannySettings::~CCannySettings()
{
}

void CCannySettings::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_THRESHHOLD1, m_dThreshhold1);
	DDX_Text(pDX, IDC_THRESHHOLD2, m_dThreshhold2);
	DDX_Text(pDX, IDC_APERTURE, m_nAperture);
	DDX_Check(pDX, IDC_L2_GRADIENT, m_bL2Gradient);
}


BEGIN_MESSAGE_MAP(CCannySettings, CDialogEx)
END_MESSAGE_MAP()


// CCannySettings message handlers
