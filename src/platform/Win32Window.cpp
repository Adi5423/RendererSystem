#include "platform/Win32Window.hpp"
#include <cstring>

Win32Window::Win32Window()
    : m_hwnd(nullptr)
    , m_hInstance(nullptr)
    , m_bmi{}
    , m_bmiInitialized(false)
{
}

Win32Window::~Win32Window() {
    if (m_hwnd) {
        DestroyWindow(m_hwnd);
        m_hwnd = nullptr;
    }
}

LRESULT CALLBACK Win32Window::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;

    case WM_CLOSE:
        DestroyWindow(hwnd);
        return 0;

    default:
        break;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

bool Win32Window::create(const char* title, int width, int height) {
    m_hInstance = GetModuleHandleA(nullptr);

    WNDCLASSEXA wc{};
    wc.cbSize = sizeof(wc);
    wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wc.lpfnWndProc = &Win32Window::WindowProc;
    wc.cbClsExtra = 0;
    wc.cbWndExtra = 0;
    wc.hInstance = m_hInstance;
    wc.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wc.lpszMenuName = nullptr;
    wc.lpszClassName = "NoPkgRenWindowClass";
    wc.hIconSm = LoadIcon(nullptr, IDI_APPLICATION);

    if (!RegisterClassExA(&wc)) {
        return false;
    }

    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;

    // Force a 4:3 client area
    int clientW = width;
    int clientH = height;

    // Compute full window size for a client area of clientW x clientH
    RECT rect{ 0, 0, clientW, clientH };
    AdjustWindowRect(&rect, style, FALSE);

    // Create non-resizable 4:3 window
    m_hwnd = CreateWindowExA(
        0,
        wc.lpszClassName,
        title,
        style,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left,
        rect.bottom - rect.top,
        nullptr,
        nullptr,
        m_hInstance,
        nullptr
    );


    if (!m_hwnd) {
        return false;
    }

    ShowWindow(m_hwnd, SW_SHOW);
    UpdateWindow(m_hwnd);

    return true;
}

bool Win32Window::processMessages() {
    MSG msg;
    while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) {
            return false;
        }
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    return true;
}

void Win32Window::initBitmapInfo(int srcWidth, int srcHeight) {
    std::memset(&m_bmi, 0, sizeof(m_bmi));
    m_bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    m_bmi.bmiHeader.biWidth = srcWidth;
    m_bmi.bmiHeader.biHeight = -srcHeight; // negative = top-down DIB
    m_bmi.bmiHeader.biPlanes = 1;
    m_bmi.bmiHeader.biBitCount = 32;         // 32-bit pixels
    m_bmi.bmiHeader.biCompression = BI_RGB;     // no compression
    m_bmiInitialized = true;
}

void Win32Window::present(const std::uint32_t* pixels, int srcWidth, int srcHeight) {
    if (!m_hwnd || !pixels) return;

    // If source resolution changed, rebuild bitmap info
    if (!m_bmiInitialized ||
        m_bmi.bmiHeader.biWidth != srcWidth ||
        m_bmi.bmiHeader.biHeight != -srcHeight)     // negative height = top-down
    {
        initBitmapInfo(srcWidth, srcHeight);
    }


    RECT clientRect;
    GetClientRect(m_hwnd, &clientRect);
    int dstWidth = clientRect.right - clientRect.left;
    int dstHeight = clientRect.bottom - clientRect.top;

    HDC hdc = GetDC(m_hwnd);

    StretchDIBits(
        hdc,
        0, 0, dstWidth, dstHeight,     // dest rectangle (window)
        0, 0, srcWidth, srcHeight,     // source rectangle (framebuffer)
        pixels,
        &m_bmi,
        DIB_RGB_COLORS,
        SRCCOPY
    );

    ReleaseDC(m_hwnd, hdc);
}
