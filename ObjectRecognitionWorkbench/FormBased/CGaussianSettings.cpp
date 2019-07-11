// CGaussianSettings.cpp : implementation file
//

#include "stdafx.h"
#include "FormBased.h"
#include "CGaussianSettings.h"
#include "afxdialogex.h"

#include <opencv2/core/base.hpp>

struct BorderTypeMap
{
	cv::BorderTypes	bt;
	LPCTSTR	name;
} btMap[] =
{
	{ cv::BORDER_CONSTANT, _T("Constant")},
	{ cv::BORDER_REPLICATE, _T("Replicate")},
	{ cv::BORDER_REFLECT, _T("Reflect")},
	{ cv::BORDER_WRAP, _T("Wrap")},
	{ cv::BORDER_REFLECT_101, _T("Reflect 101")},
	{ cv::BORDER_TRANSPARENT, _T("Transparent")},
	{ cv::BORDER_ISOLATED, _T("Isolated")},
};
// CGaussianSettings dialog
	
IMPLEMENT_DYNAMIC(CGaussianSettings, CDialogEx)

CGaussianSettings::CGaussianSettings(CSize size, double dSigmaX, double dSigmaY, int nBorderType, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_GAUSSIAN_SETTINGS, pParent)
	, m_nSizeX(size.cx)
	, m_nSizeY(size.cy)
	, m_dSigmaX(dSigmaX)
	, m_dSigmaY(dSigmaY)
	, m_strBorderType(_T("Reflect 101"))
	, m_nBorderType(nBorderType)
{
	for (auto map : btMap)
	{
		if (nBorderType == map.bt)
		{
			m_strBorderType = map.name;
			break;
		}
	}
}

CGaussianSettings::~CGaussianSettings()
{
}

void CGaussianSettings::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_SIZE_X, m_nSizeX);
	DDX_Text(pDX, IDC_SIZE_Y, m_nSizeY);
	DDX_Text(pDX, IDC_SIGMA_X, m_dSigmaX);
	DDX_Text(pDX, IDC_SIGMA_Y, m_dSigmaY);
	DDX_CBString(pDX, IDC_BORDER_TYPE, m_strBorderType);
	DDX_Control(pDX, IDC_BORDER_TYPE, m_BorderType);
}


BEGIN_MESSAGE_MAP(CGaussianSettings, CDialogEx)
	ON_CBN_SELCHANGE(IDC_BORDER_TYPE, &CGaussianSettings::OnCbnSelchangeBorderType)
END_MESSAGE_MAP()


// CGaussianSettings message handlers


BOOL CGaussianSettings::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  Add extra initialization here

	return TRUE;  // return TRUE unless you set the focus to a control
				  // EXCEPTION: OCX Property Pages should return FALSE
}


void CGaussianSettings::OnCbnSelchangeBorderType()
{
	int nIdx = m_BorderType.GetCurSel();

	CString strValue;
	m_BorderType.GetLBText(nIdx, strValue);

	for (auto map : btMap)
	{
		if (strValue == map.name)
		{
			m_nBorderType = map.bt;
			break;
		}
	}
}
