
// Build Vision Dev EnvironmentDlg.cpp : implementation file
//

#include "stdafx.h"
#include "Build Vision Dev Environment.h"
#include "Build Vision Dev EnvironmentDlg.h"
#include "HelpDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CString
GetEnv(const CString& strName)
{
	CString strValue;
	strValue.GetEnvironmentVariable(strName);
	return strValue;
}

bool
AddToSystemPath(CString strDir)
{
	HKEY hKeyEnv;
	LONG lRes = ::RegOpenKey(HKEY_LOCAL_MACHINE, _T("SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment"), &hKeyEnv);
	if (lRes != ERROR_SUCCESS)
		return false;
	DWORD dwType;
	DWORD cData = 0;
	lRes = ::RegQueryValueEx(hKeyEnv, _T("PATH"), NULL, &dwType, NULL, &cData);
	if (lRes != ERROR_SUCCESS)
		return false;

	size_t cBuff = cData + strDir.GetLength() * sizeof(TCHAR);
	BYTE* pszData = new BYTE[cBuff + 100];						// Add 100 bytes for possible semicolon, null and safety

	lRes = ::RegQueryValueEx(hKeyEnv, _T("PATH"), NULL, &dwType, (LPBYTE)pszData, &cData);
	if (lRes != ERROR_SUCCESS)
		return false;

	TCHAR* pszValue = (TCHAR*)pszData;
	size_t cValue = _tcslen(pszValue);

	if (pszValue[cValue - 1] != _T(';'))
	{
		pszData[cValue++] = _T(';');
		pszData[cValue] = _T('\000');
		cData += sizeof(TCHAR);
	}

	strDir += _T(';');
	_tcscat(pszValue, (LPCTSTR)strDir);
	pszData[cBuff] = 0;

	lRes = ::RegSetValueEx(hKeyEnv, _T("SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment"), NULL, dwType, pszData, (_tcslen(pszValue) + 1) * sizeof(TCHAR));
	return true;
}

CString
GetIncludeVersion(const CString& strDir)
{
	WIN32_FIND_DATA wfd;
	CString rVal;

	HANDLE hFind = ::FindFirstFile(strDir + _T("\\include\\*"), &wfd);

	while (hFind != INVALID_HANDLE_VALUE)
	{
		if (wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY &&
			wfd.cFileName[0] != _T('.'))
		{
			if (!rVal.IsEmpty())
				return _T("");				// Must only be a single directory
			rVal = wfd.cFileName;
		}
		if (!::FindNextFile(hFind, &wfd))
		{
			hFind = INVALID_HANDLE_VALUE;
		}
	}
	return rVal;
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CBuildVisionDevEnvironmentDlg dialog



CBuildVisionDevEnvironmentDlg::CBuildVisionDevEnvironmentDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_BUILDVISIONDEVENVIRONMENT_DIALOG, pParent)
	, m_strTargetDirectory(_T(""))
	, m_strRealSenseDirectory(_T(""))
	, m_strPCLDirectory(_T(""))
	, m_strLZ4Directory(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CBuildVisionDevEnvironmentDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_TARGET_DIRECTORY, m_strTargetDirectory);
	DDX_Text(pDX, IDC_REALSENSE_DIRECTORY, m_strRealSenseDirectory);
	DDX_Text(pDX, IDC_PCL_DIRECTORY, m_strPCLDirectory);
	DDX_Text(pDX, IDC_LZ4_DIRECTORY, m_strLZ4Directory);
}

BEGIN_MESSAGE_MAP(CBuildVisionDevEnvironmentDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BROWSE_TARGET, &CBuildVisionDevEnvironmentDlg::OnBnClickedBrowseTarget)
	ON_BN_CLICKED(IDC_BROWSE_REALSENSE, &CBuildVisionDevEnvironmentDlg::OnBnClickedBrowseRealsense)
	ON_BN_CLICKED(IDC_GO, &CBuildVisionDevEnvironmentDlg::OnBnClickedGo)
	ON_BN_CLICKED(IDC_MYHELP, &CBuildVisionDevEnvironmentDlg::OnBnClickedHelp)
END_MESSAGE_MAP()


// CBuildVisionDevEnvironmentDlg message handlers

BOOL CBuildVisionDevEnvironmentDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	TCHAR szProgramFiles86[MAX_PATH];

	SHGetSpecialFolderPath(
		0,
		szProgramFiles86,
		CSIDL_PROGRAM_FILESX86,
		FALSE);

	CCommandLineInfo rCmdInfo;

	::AfxGetApp()->ParseCommandLine(rCmdInfo);

//	m_strTargetDirectory = AfxGetApp()->GetProfileString(_T("Directories"), _T("Target"), _T(""));
	m_strTargetDirectory = rCmdInfo.m_strFileName;
//	::AfxMessageBox(m_strTargetDirectory);

	int nIdx;
	while ((nIdx = m_strTargetDirectory.Find(_T('\"'))) != -1)
	{
		//TCHAR szX[100];
		//_itot(nIdx, szX, 10);

		//::AfxMessageBox(szX);
		m_strTargetDirectory = m_strTargetDirectory.Left(nIdx) + m_strTargetDirectory.Mid(nIdx + 1);
	}

//	::AfxMessageBox(m_strTargetDirectory);

	m_strRealSenseDirectory = CString(szProgramFiles86) + _T("\\Intel RealSense SDK 2.0");
	m_strPCLDirectory = ::GetEnv(_T("PCL_ROOT"));
	if (m_strPCLDirectory.IsEmpty())
		m_strPCLDirectory = _T("PCL_ROOT not set.  Is PCL installed?");
	m_strLZ4Directory = ::GetEnv(_T("LZ4_ROOT"));
	if (m_strLZ4Directory.IsEmpty())
		m_strLZ4Directory = _T("LZ4_ROOT not set.  Is LZ4 installed?");

	m_strPCLIncludeDirectory = ::GetIncludeVersion(m_strPCLDirectory);
	m_strVTKIncludeDirectory = ::GetIncludeVersion(m_strPCLDirectory + _T("\\3rdParty\\VTK"));
	m_strBoostIncludeDirectory = ::GetIncludeVersion(m_strPCLDirectory + _T("\\3rdParty\\Boost"));

	UpdateData(FALSE);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CBuildVisionDevEnvironmentDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CBuildVisionDevEnvironmentDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CBuildVisionDevEnvironmentDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CBuildVisionDevEnvironmentDlg::OnBnClickedBrowseTarget()
{
	CFolderPickerDialog dlg(NULL, OFN_FILEMUSTEXIST, this);
	if (dlg.DoModal() == IDOK) {
		m_strTargetDirectory = dlg.GetFolderPath();
		UpdateData(FALSE);
		AfxGetApp()->WriteProfileString(_T("Directories"), _T("Target"), m_strTargetDirectory);
	}
}


void CBuildVisionDevEnvironmentDlg::OnBnClickedBrowseRealsense()
{
	CFolderPickerDialog dlg(NULL, OFN_FILEMUSTEXIST, this);
	if (dlg.DoModal() == IDOK) {
		m_strRealSenseDirectory = dlg.GetFolderPath();
		UpdateData(FALSE);
		AfxGetApp()->WriteProfileString(_T("Directories"), _T("RealSense"), m_strRealSenseDirectory);
	}
}


//typedef struct tagCopy {
//	LPCTSTR	szSourcePattern;
//	LPCTSTR	szDestDir;
//	LPCTSTR	szMustContain;
//	LPCTSTR	szMustNotContain;
//	BOOL	bRecurse;
//} COPY, *LPCOPY;
//
//COPY toCopy[] = {
//	{ _T("include\\librealsense2\\*"), 	_T("RealSense\\include\\librealsense2"), NULL, NULL, TRUE }
//
//};
//
//#define	N_COPYS	(sizeof(toCopy)/sizeof(toCopy[0]))

LPCTSTR szProps1 =
_T("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n")
_T("<Project ToolsVersion=\"4.0\" xmlns=\"http://schemas.microsoft.com/developer/msbuild/2003\">\n")
_T("  <ImportGroup Label=\"PropertySheets\" />\n")
_T("  <PropertyGroup Label=\"UserMacros\">\n")
_T("    <librealsenseSDK>$(ProgramFiles)\\Intel RealSense SDK 2.0</librealsenseSDK>\n")
_T("    <PCLDebug Condition=\"'$(Configuration)'=='Debug'\">_debug</PCLDebug>\n")
_T("    <PCLDebug Condition=\"'$(Configuration)'!='Debug'\">_release</PCLDebug>\n")
_T("    <VTKDebug Condition=\"'$(Configuration)'=='Debug'\">-gd</VTKDebug>\n")
_T("    <VTKDebug Condition=\"'$(Configuration)'!='Debug'\"></VTKDebug>\n")
_T("    <BoostDebug Condition=\"'$(Configuration)'=='Debug'\">-gd</BoostDebug>\n")
_T("    <BoostDebug Condition=\"'$(Configuration)'!='Debug'\"></BoostDebug>\n")
_T("    <OpenCVDebug Condition=\"'$(Configuration)'=='Debug'\">d</OpenCVDebug>\n")
_T("    <OpenCVDebug Condition=\"'$(Configuration)'!='Debug'\"></OpenCVDebug>\n");

LPCTSTR szProps2 =
_T("  </PropertyGroup>\n")
_T("  <PropertyGroup />\n")
_T("  <ItemDefinitionGroup>\n")
_T("    <ClCompile>\n")
_T("      <AdditionalIncludeDirectories>")
_T("      $(librealsenseSDK)\\include;\n")
_T("	  $(librealsenseSDK)\\third-party\\;\n")
_T("      $(PCL_ROOT)\\include\\$(PCL_VER);\n")
_T("      $(PCL_ROOT)\\3rdParty\\Eigen\\eigen3;\n")
_T("      $(PCL_ROOT)\\3rdParty\\FLANN\\include;\n")
_T("      $(PCL_ROOT)\\3rdParty\\VTK\\include\\$(VTK_VER);\n")
_T("      $(PCL_ROOT)\\3rdParty\\Boost\\include\\$(BOOST_VER);\n")
_T("      $(librealsenseSDK)\\third-party\\glfw-imgui\\include;\n")
_T("      $(OPENCV_ROOT)\\include;\n")
_T("	  %(AdditionalIncludeDirectories)</AdditionalIncludeDirectories>\n")
_T("    </ClCompile>\n")
_T("    <Link>\n")
_T("      <AdditionalLibraryDirectories>\n")
_T("		$(librealsenseSDK)\\lib\\$(PlatformShortName);\n")
_T("		$(librealsenseSDK)\\samples\\$(PlatformShortName)\\$(Configuration);\n")
_T("        $(PCL_ROOT)\\lib;\n")
_T("        $(PCL_ROOT)\\3rdParty\\Boost\\lib;\n")
_T("        $(PCL_ROOT)\\3rdParty\\VTK\\lib;\n")
_T("		$(LZ4_ROOT)\\$(PlatformShortName)\\$(Configuration);\n")
_T("        $(OPENCV_ROOT)\\x64\\vc15\\lib;\n")
_T("		%(AdditionalLibraryDirectories)\n")
_T("	  </AdditionalLibraryDirectories>\n")
_T("      <AdditionalDependencies>\n")
_T("		realsense2.lib;\n")
_T("		liblz4_static.lib;\n")
_T("		glfw-imgui.lib;\n")
_T("        glu32.lib;\n")
_T("        opengl32.lib;\n");

LPCTSTR szProps3 =
_T("		%(AdditionalDependencies)\n")
_T("	  </AdditionalDependencies>\n")
_T("      <ShowProgress>LinkVerbose</ShowProgress>\n")
_T("    </Link>\n")
_T("    <PostBuildEvent>\n")
_T("      <Command>xcopy /y \"$(librealsenseSDK)\\bin\\$(PlatformShortName)\\realsense2.dll\" \"$(OutDir)\"</Command>\n")
_T("    </PostBuildEvent>\n")
_T("    <PostBuildEvent>\n")
_T("      <Message>Copy Intel RealSense SDK 2.0 shared module next to the application</Message>\n")
_T("    </PostBuildEvent>\n")
_T("  </ItemDefinitionGroup>\n")
_T("  <ItemGroup>\n")
_T("    <BuildMacro Include=\"librealsenseSDK\">\n")
_T("      <Value>$(librealsenseSDK)</Value>\n")
_T("    </BuildMacro>\n")
_T("  </ItemGroup>\n")
_T("</Project>\n");



void AddLibs(CStdioFile& file, CString strDir, CString strDebugSuffix, CString strDebugVariable)
{
	WIN32_FIND_DATA wfd;

// Look for files of form *.lib

	HANDLE hFind = ::FindFirstFile(strDir + _T("\\*.lib"), &wfd);
	if (hFind == INVALID_HANDLE_VALUE)
		return;

	while (hFind != INVALID_HANDLE_VALUE)
	{
		CString strLine;
		strLine.Empty();

// If strDebugSuffix is "", there's only one version of the file.  Write its name

		if (strDebugSuffix.IsEmpty())
		{
			strLine.Format(_T("        %s;\n"), wfd.cFileName);
		}

// If strDebugSuffix is not "", then we look for a file containing that "suffix" somewhere in its name.
// The nomenclature "suffix" is left over from an earlier version of this code that required the debug
// string to be a trailing suffix (just before the .lib extension).

		else
		{
			CString strFile = wfd.cFileName;
			int nIdx = strFile.Find(strDebugSuffix);
			if (nIdx != -1)
			{
// Replace the debug suffix with a variable (strDebugVariable).  We end up with, for example:
// pcl_visualization$(PCLDebug).lib

				strFile = strFile.Left(nIdx) + strDebugVariable + strFile.Mid(nIdx + strDebugSuffix.GetLength());
				strLine.Format(_T("        %s;\n"), strFile);
			}
		}
		if (!strLine.IsEmpty())
			file.WriteString(strLine);
		if (!::FindNextFile(hFind, &wfd))
		{
			::FindClose(hFind);
			hFind = INVALID_HANDLE_VALUE;
		}
			
	}
}

CString strPCL_ROOT;
CString strOPENNI2_INCLUDE64;
CString strOPENNI2_LIB64;
CString strOPENNI2_REDIST64;
CString strOPENCV_ROOT;

typedef struct tagEnv
{
	LPCTSTR		pszName;
	CString&	strValue;
} ENV;

ENV EnvVbls[] = {
	{ _T("PCL_ROOT"), strPCL_ROOT },
	{ _T("OPENNI2_INCLUDE64"), strOPENNI2_INCLUDE64 },
	{ _T("OPENNI2_LIB64"), strOPENNI2_LIB64 },
	{ _T("OPENNI2_REDIST64"), strOPENNI2_REDIST64 },
//	{ _T("OPENCV_VER"), strOPENCV_VER },
	{ _T("OPENCV_ROOT"), strOPENCV_ROOT }
};

#define	N_ENVS		(sizeof(EnvVbls)/sizeof(EnvVbls[0]))

LPCTSTR szDirectories[] = {
	_T("$(PCL_ROOT)\\bin"),
	_T("$(PCL_ROOT)\\lib"),
	_T("$(PCL_ROOT)\\3rdParty\\Boost\\lib"),
	_T("$(PCL_ROOT)\\3rdParty\\VTK\\lib"),
	_T("$(PCL_ROOT)\\include\\$(PCL_VER)"),
	_T("$(PCL_ROOT)\\3rdParty\\Eigen\\eigen3"),
	_T("$(PCL_ROOT)\\3rdParty\\FLANN\\include"),
	_T("$(PCL_ROOT)\\3rdParty\\VTK\\include\\$(VTK_VER)"),
	_T("$(PCL_ROOT)\\3rdParty\\Boost\\include\\$(BOOST_VER)"),
	_T("$(librealsenseSDK)\\lib\\$(PlatformShortName)"),
	_T("$(librealsenseSDK)\\samples\\$(PlatformShortName)\\$(Configuration)"),
	_T("$(librealsenseSDK)\\include"),
	_T("$(librealsenseSDK)\\third-party"),
	_T("$(librealsenseSDK)\\third-party\\glfw-imgui\\include"),
	_T("$(OPENCV_ROOT")
};

#define	N_DIRS		(sizeof(szDirectories)/sizeof(szDirectories[0]))

LPCTSTR pszPlatforms[2] = { _T("x86"), _T("x64") };
LPCTSTR pszConfigs[2] = { _T("Debug"), _T("Release") };

void CBuildVisionDevEnvironmentDlg::OnBnClickedGo()
{
	BOOL bError = FALSE;
// Test for correct installation of components

/*	PCL
		PCL_ROOT must be set
		PATH must contain $(PCL_ROOT)\bin value
		
		OPENNI2_INCLUDE64 must be set
		OPENNI2_LIB64 must be set
		OPENNI2_REDIST64 must be set

		PATH must contain $(OPENNI2_REDIST64) value

		$(PCL_ROOT)\bin 
		$(PCL_ROOT)\lib 
		$(PCL_ROOT)\3rdParty\Boost\lib 
		$(PCL_ROOT)\3rdParty\VTK\lib
		$(PCL_ROOT)\include\pcl-1.8
		$(PCL_ROOT)\3rdParty\Eigen\eigen3
		$(PCL_ROOT)\3rdParty\FLANN\include
		$(PCL_ROOT)\3rdParty\VTK\include\vtk-8.0
		$(PCL_ROOT)\3rdParty\Boost\include\boost-1_64

		$(librealsenseSDK)\lib\$(PlatformShortName)
		$(librealsenseSDK)\samples\$(PlatformShortName)\$(Configuration)

		$(librealsenseSDK)\include
		$(librealsenseSDK)\third-party
		$(librealsenseSDK)\third-party\glfw-imgui\include
 */

// Verify the environment variables

	for (int i = 0; i < N_ENVS; i++)
	{
		EnvVbls[i].strValue = ::GetEnv(EnvVbls[i].pszName);
		if (EnvVbls[i].strValue.IsEmpty())
		{
			::AfxMessageBox(CString(_T("Environment variable not set: ")) + EnvVbls[i].pszName, MB_ICONEXCLAMATION | MB_OK);
			bError = TRUE;
		}
	}

// Verify the path contents:

	CString strPath = ::GetEnv(_T("PATH"));
	//int x = strPath.GetLength();

	//::AfxMessageBox(strPath, MB_ICONINFORMATION | MB_OK);

	if (strPath.Find(strPCL_ROOT + _T("\\bin")) == -1 && strPath.Find(_T("%PCL_ROOT%\\bin")) == -1)
	{
		::AfxMessageBox(CString(_T("PATH should contain ")) + strPCL_ROOT + _T("\\bin or %PCL_ROOT%\\bin, but doesn't."), MB_ICONEXCLAMATION | MB_OK);
//		if (::AfxMessageBox(CString(_T("PATH should contain ")) + strPCL_ROOT + _T("\\bin or %PCL_ROOT%\\bin, but doesn't\n\nShall I add it?"), MB_ICONEXCLAMATION | MB_YESNO) == IDYES)
		//{
		//	bError = !AddToSystemPath(strPCL_ROOT + _T("\\bin"));
		//}
		//else
			bError = TRUE;
	}

	if (strPath.Find(strOPENNI2_REDIST64) == -1)
	{
		::AfxMessageBox(CString(_T("PATH should contain ")) + strOPENNI2_REDIST64 + _T(", but doesn't."), MB_ICONEXCLAMATION | MB_OK);
//		if (::AfxMessageBox(CString(_T("PATH should contain ")) + strOPENNI2_REDIST64 + _T(", but doesn't\n\nShall I add it?"), MB_ICONEXCLAMATION | MB_YESNO) == IDYES)
		//{
		//	bError = !AddToSystemPath(strOPENNI2_REDIST64);
		//}
		//else
			bError = TRUE;
	}

	if (strPath.Find(strOPENCV_ROOT + _T("\\x64\\vc15\\bin")) == -1)
	{
		::AfxMessageBox(CString(_T("PATH should contain ")) + strOPENCV_ROOT + _T("\\x64\\vc15\\bin") + _T(", but doesn't."), MB_ICONEXCLAMATION | MB_OK);
		//		if (::AfxMessageBox(CString(_T("PATH should contain ")) + strOPENCV_ROOT + _T("\\x64\\vc15\\bin") + _T(", but doesn't\n\nShall I add it?"), MB_ICONEXCLAMATION | MB_YESNO) == IDYES)
				//{
				//	bError = !AddToSystemPath(strOPENCV_ROOT + _T("\\x64\\vc15\\bin"));
				//}
				//else
		bError = TRUE;
	}

	// Verify the directories

	for (int i = 0; i < N_DIRS; i++)
	{
		CString strDir = szDirectories[i];
		int nPSN = -1;		// Location of PlatformShortName - no space reserved, must insert to use
		int nCfg = -1;		// Location of Configuration - no space reserved, must insert to use

		int nIdx;
		while ((nIdx = strDir.Find(_T("$("))) != -1)
		{
			if (strDir.Mid(nIdx + 2, 8) == _T("PCL_ROOT"))
			{
				strDir = strDir.Left(nIdx) + strPCL_ROOT + strDir.Mid(nIdx + 11);
			}
			else if (strDir.Mid(nIdx + 2, 15) == _T("librealsenseSDK"))
			{
				strDir = strDir.Left(nIdx) + m_strRealSenseDirectory + strDir.Mid(nIdx + 18);
			}
			else if (strDir.Mid(nIdx + 2, 17) == _T("PlatformShortName"))
			{
				strDir = strDir.Left(nIdx) + strDir.Mid(nIdx + 20);
				nPSN = nIdx;
			}
			else if (strDir.Mid(nIdx + 2, 13) == _T("Configuration"))
			{
				strDir = strDir.Left(nIdx) + strDir.Mid(nIdx + 16);
				nCfg = nIdx;
			}
			else if (strDir.Mid(nIdx + 2, 7) == _T("PCL_VER"))
			{
				strDir = strDir.Left(nIdx) + m_strPCLIncludeDirectory + strDir.Mid(nIdx + 10);
			}
			else if (strDir.Mid(nIdx + 2, 7) == _T("VTK_VER"))
			{
				strDir = strDir.Left(nIdx) + m_strVTKIncludeDirectory + strDir.Mid(nIdx + 10);
			}
			else if (strDir.Mid(nIdx + 2, 9) == _T("BOOST_VER"))
			{
				strDir = strDir.Left(nIdx) + m_strBoostIncludeDirectory + strDir.Mid(nIdx + 12);
			}
			else if (strDir.Mid(nIdx + 2, 11) == _T("OPENCV_ROOT"))
			{
				strDir = strDir.Left(nIdx) + strOPENCV_ROOT + strDir.Mid(nIdx + 14);
			}
		}

		CStringArray strDirs;
		CString strCandidate;
		for (int j = 0; j < 2; j++)
		{
			int nOffset = 0;

			if (nPSN != -1)
			{
				strCandidate = strDir.Left(nPSN) + pszPlatforms[j] + strDir.Mid(nPSN);
				nOffset = wcslen(pszPlatforms[j]);
			}
			else
				strCandidate = strDir;

			for (int k = 0; k < 2; k++)
			{
				if (nCfg != -1)
					strDirs.Add(strCandidate.Left(nCfg+nOffset) + pszConfigs[k] + strCandidate.Mid(nCfg+nOffset));
				else
					strDirs.Add(strCandidate);

				if (nCfg == -1)
					break;
			}
			if (nPSN == -1)
				break;
		}
		for (int j = 0; j < strDirs.GetSize(); j++)
		{
			CString strTest = strDirs[j];
			if (::GetFileAttributes(strTest) != FILE_ATTRIBUTE_DIRECTORY)
			{
				if (strTest.Find(_T("\\samples\\")) != -1)
				{
					::AfxMessageBox(strTest + _T(" does not exist or is not a directory.  This directory contains GLFW-IMGUI, which consists of a bunch of useful helper functions used by the RealSense samples.  It is not strictly necessary to use the RealSense libraries.  You can populate this directory by building the RealSense samples."), MB_ICONEXCLAMATION | MB_OK);
				}
				else
				{ 
					::AfxMessageBox(strTest + _T(" does not exist or is not a directory"), MB_ICONEXCLAMATION | MB_OK);
					bError = TRUE;
				}
			}
		}
	}

	if (!bError)
	{
		CString strTarget = m_strTargetDirectory + _T("\\LMVision.props");
		if (::GetFileAttributes(strTarget) != INVALID_FILE_ATTRIBUTES)
		{
			if (::AfxMessageBox(_T("LMVision.props exists.  Do you want to overwrite it?"), MB_ICONQUESTION | MB_YESNO) == IDNO)
				return;
		}
		CStdioFile file(strTarget, CFile::modeWrite | CFile::typeText | CFile::modeCreate);

		CString strVbl;

		file.WriteString(szProps1);

		strVbl.Format(_T("    <PCL_VER>%s</PCL_VER>\n"), m_strPCLIncludeDirectory);
		file.WriteString(strVbl);

		strVbl.Format(_T("    <VTK_VER>%s</VTK_VER>\n"), m_strVTKIncludeDirectory);
		file.WriteString(strVbl);

		strVbl.Format(_T("    <BOOST_VER>%s</BOOST_VER>\n"), m_strBoostIncludeDirectory);
		file.WriteString(strVbl);


		file.WriteString(szProps2);

		AddLibs(file, m_strPCLDirectory + _T("\\lib"), _T("_debug"), _T("$(PCLDebug)"));
		AddLibs(file, m_strPCLDirectory + _T("\\3rdParty\\Boost\\lib"), _T("-gd"), _T("$(BoostDebug)"));
		AddLibs(file, m_strPCLDirectory + _T("\\3rdParty\\VTK\\lib"), _T("-gd"), _T("$(VTKDebug)"));

		AddLibs(file, strOPENCV_ROOT + _T("\\x64\\vc15\\lib"), _T("d.lib"), _T("$(OPENCVDebug).lib"));

		file.WriteString(szProps3);

		::AfxMessageBox(_T("Done!"), MB_ICONINFORMATION | MB_OK);
	}
	else
		::AfxMessageBox(_T(".props file not built due to fatal errors"), MB_ICONINFORMATION | MB_OK);
}


//BOOL CBuildVisionDevEnvironmentDlg::BuildDirectoryTree()
//{
//	for (int i = 0; i < N_DIRS; i++)
//	{
//		CString strDir = m_strTargetDirectory + "\\" + CString(szDirectories[i]);
//		BOOL bRes = ::CreateDirectory(strDir, NULL);
//		DWORD dwRes = ::GetLastError();
//		if (!bRes && dwRes != ERROR_ALREADY_EXISTS)
//		{
//			AfxMessageBox(CString(_T("Couldn't create ")) + strDir, MB_ICONEXCLAMATION | MB_OK);
//		}
//	}
//	return 0;
//}


void CBuildVisionDevEnvironmentDlg::OnBnClickedHelp()
{
	CHelpDlg dlg;

	dlg.DoModal();
}
