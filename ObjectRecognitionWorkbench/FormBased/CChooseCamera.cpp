// CChooseCamera.cpp : implementation file
//

#include "stdafx.h"
#include "FormBased.h"
#include "CChooseCamera.h"
#include "afxdialogex.h"


// CChooseCamera dialog

IMPLEMENT_DYNAMIC(CChooseCamera, CDialogEx)

CChooseCamera::CChooseCamera(std::vector<rs2::device>& depthCameras, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_CHOOSE_CAMERA, pParent)
	, m_depthCameras(depthCameras)
	, m_nSelectedCamera(-1)
{

}

CChooseCamera::~CChooseCamera()
{
}

void CChooseCamera::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CAMERAS, m_ctrlCameras);
}


BEGIN_MESSAGE_MAP(CChooseCamera, CDialogEx)
	ON_CBN_CLOSEUP(IDC_CAMERAS, &CChooseCamera::OnCbnCloseupCameras)
END_MESSAGE_MAP()


// CChooseCamera message handlers


BOOL CChooseCamera::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	for (auto&& dev : m_depthCameras)
	{
		CString strCameraName(dev.get_info(RS2_CAMERA_INFO_NAME));
		CString strCameraSerialNumber(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

		m_ctrlCameras.AddString(strCameraSerialNumber + _T("  ") + strCameraName);
	}

	return TRUE;  // return TRUE unless you set the focus to a control
				  // EXCEPTION: OCX Property Pages should return FALSE
}


void CChooseCamera::OnCbnCloseupCameras()
{
	m_nSelectedCamera = m_ctrlCameras.GetCurSel();
}
