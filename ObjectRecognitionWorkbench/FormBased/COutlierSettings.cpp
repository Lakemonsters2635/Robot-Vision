// COutlierSettings.cpp : implementation file
//

#include "stdafx.h"
#include "FormBased.h"
#include "COutlierSettings.h"
#include "afxdialogex.h"


// COutlierSettings dialog

IMPLEMENT_DYNAMIC(COutlierSettings, CDialogEx)

COutlierSettings::COutlierSettings(long lMeanK, double dStdDevMultiplier, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_OUTLIER_SETTINGS, pParent)
	, m_lMeanK(lMeanK)
	, m_dStdDevMultiplier(dStdDevMultiplier)
{

}

COutlierSettings::~COutlierSettings()
{
}

void COutlierSettings::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_MEAN_K, m_lMeanK);
	DDX_Text(pDX, IDC_STD, m_dStdDevMultiplier);
}


BEGIN_MESSAGE_MAP(COutlierSettings, CDialogEx)
//	ON_EN_CHANGE(IDC_MEAN_K, &COutlierSettings::OnEnChangeMeanK)
END_MESSAGE_MAP()


// COutlierSettings message handlers


