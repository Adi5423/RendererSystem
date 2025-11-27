#pragma once

// Minimal Win32 window wrapper for showing a software framebuffer.
// No external libs, just Win32 + GDI.

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdint>

class Win32Window {
public:
    Win32Window();
    ~Win32Window();

    // Create and show the window.
    // width, height are the initial outer window size (not exact client size).
    bool create(const char* title, int width, int height);

    // Handle pending Win32 messages.
    // Returns false when WM_QUIT is received (i.e., user closed the window).
    bool processMessages();

    // Present a framebuffer to the window using StretchDIBits.
    // pixels: pointer to width*height 32-bit pixels (0x00BBGGRR).
    void present(const std::uint32_t* pixels, int srcWidth, int srcHeight);

    HWND getHWND() const { return m_hwnd; }

private:
    HWND       m_hwnd;
    HINSTANCE  m_hInstance;
    BITMAPINFO m_bmi;
    bool       m_bmiInitialized;

    void initBitmapInfo(int srcWidth, int srcHeight);

    // Static window procedure
    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
};
