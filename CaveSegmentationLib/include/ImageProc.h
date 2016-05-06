#pragma once

#include "Options.h"

#include <windows.h>
#include <gdiplus.h>
#pragma comment (lib, "Gdiplus.lib")
#undef min
#undef max

#ifdef DRAW_DEBUG_IMAGES
	extern CLSID pngEncoder;
	extern ULONG_PTR gdiplusToken;
#endif

extern int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);

extern void StartImageProc();

extern void StopImageProc();

extern void SavePNG(Gdiplus::Image* image, const wchar_t* filename);