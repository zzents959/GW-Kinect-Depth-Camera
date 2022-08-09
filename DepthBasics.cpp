//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "Windows.h"
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "DepthBasics.h"
#include <math.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>


using namespace cv;
using namespace std;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CDepthBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CDepthBasics::CDepthBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_bSaveScreenshot(FALSE),
    m_pKinectSensor(NULL),
    m_pDepthFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pDrawDepth(NULL),
    m_pDepthRGBX(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
}
  

/// <summary>
/// Destructor
/// </summary>
CDepthBasics::~CDepthBasics()
{
    // clean up Direct2D renderer
    if (m_pDrawDepth)
    {
        delete m_pDrawDepth;
        m_pDrawDepth = NULL;
    }

    if (m_pDepthRGBX)
    {
        delete [] m_pDepthRGBX;
        m_pDepthRGBX = NULL;
    }

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CDepthBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"DepthBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CDepthBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

int frameCounting = 1;
time_t startingTime = time(0);
double deltaTime = 3;
/// <summary>
/// Main processing function
/// </summary>
void CDepthBasics::Update()
{
    if (!m_pDepthFrameReader)
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;
        double seconds_since_start = difftime(time(0), startingTime);

        hr = pDepthFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        }

        if (SUCCEEDED(hr))
        {
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
            //// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
        }

        if (SUCCEEDED(hr))
        {
            ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
        }

        if (SUCCEEDED(hr) && (seconds_since_start >= deltaTime))
        {
            RecordFrames();
        }

        if (SUCCEEDED(hr) && (m_bSaveScreenshot))
        {
            Contours();
        }

        SafeRelease(pFrameDescription);
    }
    SafeRelease(pDepthFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CDepthBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CDepthBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CDepthBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawDepth = new ImageRenderer();
            HRESULT hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cDepthWidth, cDepthHeight, cDepthWidth * sizeof(RGBQUAD)); 
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
            if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                m_bSaveScreenshot = true;
            }
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        SafeRelease(pDepthFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
void CDepthBasics::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
    if (m_hWnd)
    {
        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }

    // Make sure we've received valid data
    if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
    {
        RGBQUAD* pRGBX = m_pDepthRGBX;

        // end pixel is start + width*height - 1
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

        while (pBuffer < pBufferEnd)
        {
            USHORT depth = *pBuffer;


            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap."
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.;

            // float blends colors, int hard seperate colors
            // w = width of spectrum in mm
            int w = 500;
            int a = ((15.71 * depth) / (3 * w)) + 1.57;
            int rnoclamp = sin(a) * 127 + 130;
            int gnoclamp = sin(a - 6.28 / 3) * 127 + 130;
            int bnoclamp = sin(a - 12.57 / 3) * 127 + 130;
            
            BYTE r = static_cast<BYTE> ((rnoclamp < 0) ? 0 : ((rnoclamp > 255) ? 255 : rnoclamp));
            BYTE g = static_cast<BYTE> ((gnoclamp < 0) ? 0 : ((gnoclamp > 255) ? 255 : gnoclamp));
            BYTE b = static_cast<BYTE> ((bnoclamp < 0) ? 0 : ((bnoclamp > 255) ? 255 : bnoclamp));
            
            pRGBX->rgbRed = r;
            pRGBX->rgbGreen = g;
            pRGBX->rgbBlue = b;

            ++pRGBX;
            ++pBuffer;
        }

        // Draw the data with Direct2D
        m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CDepthBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Get the name of the file where screenshot will be stored.
/// </summary>
/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT CDepthBasics::GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize, int value)
{
    WCHAR* pszKnownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_ProgramFiles, 0, NULL, &pszKnownPath);

    if (SUCCEEDED(hr))
    {
        if (value >= 1)
        {
            int frameCount = value;
            StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\Kinect-Depth-Camera\\frames\\frame_%d.bmp", pszKnownPath, frameCount);
        }
        if (value == 0)
        {
            // Get the time
            WCHAR szTimeString[MAX_PATH];
            GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

            // File name will be KinectScreenshotDepth-HH-MM-SS.bmp
            StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\Kinect-Depth-Camera\\results\\Kinect-Depth-%s.bmp", pszKnownPath, szTimeString);
        }
        
    }

    if (pszKnownPath)
    {
        CoTaskMemFree(pszKnownPath);
    }

    return hr;
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
    DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}

/// <summary>
/// Records frames over duration (deltaTime) as bit maps
/// </summary>
/// <returns></returns>
HRESULT CDepthBasics::RecordFrames()
{
    WCHAR framePath[MAX_PATH];

    // Retrieve the path to My Photos
    GetScreenshotFileName(framePath, _countof(framePath), frameCounting);

    HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pDepthRGBX), 512, 424, sizeof(RGBQUAD) * 8, framePath);

    frameCounting += 1;
    startingTime = time(nullptr);

    return hr;
}

/// <summary>
/// Draws and displays current contours
/// </summary>
/// <returns></returns>
HRESULT CDepthBasics::Contours()
{
    // Retrieve the path to My Photos
    WCHAR szScreenshotPath[MAX_PATH];
    GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath), 0);

    // Write out the bitmap to disk
    HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pDepthRGBX), 512, 424, sizeof(RGBQUAD) * 8, szScreenshotPath);

    WCHAR szStatusMessage[64 + MAX_PATH];
    if (SUCCEEDED(hr))
    {
        // Set the status bar to show where the screenshot was saved
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Contour maps saved to %s", szScreenshotPath);
    }
    else
    {
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Failed to write contour maps to %s", szScreenshotPath);
    }

    SetStatusMessage(szStatusMessage, 5000, true);

    /** Contour Drawing **/
    // convert from wide char to narrow char array
    char inputContour[260];
    char DefChar = ' ';
    WideCharToMultiByte(CP_ACP, 0, szScreenshotPath, -1, inputContour, 260, &DefChar, NULL);

    // A string  using the char* constructor.
    string ss(inputContour);

    // read the image           
    Mat image = imread(inputContour);
    imwrite("C:\\Program Files\\Kinect-Depth-Camera\\results\\unedited_image.png", image);
    // convert the image to grayscale format
    Mat img_gray;
    cvtColor(image, img_gray, COLOR_BGR2GRAY);
    imwrite("C:\\Program Files\\Kinect-Depth-Camera\\results\\img_gray.png", img_gray);
    imshow("Display window", img_gray);
    waitKey(0);

    // mats to hold binary thresholding
    Mat thresh1;
    Mat thresh2;
    Mat thresh3;
    Mat thresh4;
    Mat thresh5;

    // apply binary thresholding and show mats
    threshold(img_gray, thresh1, 170, 255, THRESH_BINARY);
    threshold(img_gray, thresh2, 80, 255, THRESH_BINARY);
    threshold(img_gray, thresh3, 100, 255, THRESH_BINARY);
    threshold(img_gray, thresh4, 130, 255, THRESH_BINARY);
    threshold(img_gray, thresh5, 140, 255, THRESH_BINARY);
    imshow("Display window 1", thresh1);
    imshow("Display window 2", thresh2);
    imshow("Display window 3", thresh3);
    imshow("Display window 4", thresh4);
    imshow("Display window 5", thresh5);
    waitKey(0);
    destroyAllWindows();

    // detect the contours on the binary images
    vector<vector<Point>> contours1;
    vector<vector<Point>> contours2;
    vector<vector<Point>> contours3;
    vector<vector<Point>> contours4;
    vector<vector<Point>> contours5;
    vector<Vec4i> hierarchy;
    findContours(thresh1, contours1, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresh2, contours2, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresh3, contours3, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresh4, contours4, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours(thresh5, contours5, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // draw contours on the original image
    Mat image_copy = image.clone();
    Mat contourMat = Mat::zeros(Size(image.cols, image.rows), CV_8UC1);
    drawContours(image_copy, contours1, -1, Scalar(0, 0, 0), 2);
    drawContours(image_copy, contours2, -1, Scalar(0, 0, 0), 2);
    drawContours(image_copy, contours3, -1, Scalar(0, 0, 0), 2);
    drawContours(image_copy, contours4, -1, Scalar(0, 0, 0), 2);
    drawContours(image_copy, contours5, -1, Scalar(0, 0, 0), 2);

    // draw contours on blank mat
    drawContours(contourMat, contours1, -1, Scalar(255, 255, 255), 2);
    drawContours(contourMat, contours2, -1, Scalar(255, 255, 255), 2);
    drawContours(contourMat, contours3, -1, Scalar(255, 255, 255), 2);
    drawContours(contourMat, contours4, -1, Scalar(255, 255, 255), 2);
    drawContours(contourMat, contours5, -1, Scalar(255, 255, 255), 2);

    // show results
    imshow("Contours Overlay", image_copy);
    imshow("Topography", contourMat);
    waitKey(0);
    imwrite("C:\\Program Files\\Kinect-Depth-Camera\\results\\topography.jpg", contourMat);
    imwrite("C:\\Program Files\\Kinect-Depth-Camera\\results\\contours_with_image.jpg", image_copy);
    destroyAllWindows();

    /**
    // 3D visulaizer test
    open3d::geometry::Image depth_map;
    open3d::geometry::Image color_map;
    open3d::geometry::RGBDImage rgbd_image;
    auto intrinsic_camera = open3d::camera::PinholeCameraIntrinsic::PinholeCameraIntrinsic(open3d::camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault);
    auto extrinsic_camera = Eigen::Matrix4d::Identity();

    open3d::io::ReadImage("C:\\Users\\Caden\\Desktop\\Grain Weevil\\Kinect-Depth-Camera\\results\\img_gray.png", depth_map); //my guess is that the color and depth are not correct
    open3d::io::ReadImage("C:\\Users\\Caden\\Desktop\\Grain Weevil\\Kinect-Depth-Camera\\results\\unedited_image.png", color_map);
    rgbd_image.CreateFromColorAndDepth(color_map, depth_map, 1000, 3, true);

    if (rgbd_image.IsEmpty() == true) {
        imshow("Display window 1", thresh1);
        waitKey();
    }

    auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(rgbd_image, intrinsic_camera, extrinsic_camera, true);
    open3d::visualization::DrawGeometries({ pcd });
    **/

    // toggle off so we don't save a screenshot again next frame
    m_bSaveScreenshot = false;

    return hr;
}

/// <summary>
/// Creates Video using FFMPEG from saved frames
/// </summary>
void CDepthBasics::CreateVideo(int numberOfFrames)
{
    // Paste this in CMD when in frames folder
    // ffmpeg -r 2 -i frame_%d.bmp -c:v libx264 -vf "fps = 25, format = yuv420p" frame.mp4
}
