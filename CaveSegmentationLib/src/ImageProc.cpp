#include "ImageProc.h"

#ifdef DRAW_DEBUG_IMAGES
CLSID pngEncoder;
ULONG_PTR gdiplusToken;
#endif

int GetEncoderClsid(const WCHAR * format, CLSID * pClsid)
{
	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes

	Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;

	Gdiplus::GetImageEncodersSize(&num, &size);
	if (size == 0)
		return -1;  // Failure

	pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
	if (pImageCodecInfo == NULL)
		return -1;  // Failure

	GetImageEncoders(num, size, pImageCodecInfo);

	for (UINT j = 0; j < num; ++j)
	{
		if (wcscmp(pImageCodecInfo[j].MimeType, format) == 0)
		{
			*pClsid = pImageCodecInfo[j].Clsid;
			free(pImageCodecInfo);
			return j;  // Success
		}
	}

	free(pImageCodecInfo);
	return -1;  // Failure
}

void StartImageProc()
{
#ifdef DRAW_DEBUG_IMAGES
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
	Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	GetEncoderClsid(L"image/png", &pngEncoder);
#endif	
}

void StopImageProc()
{
#ifdef DRAW_DEBUG_IMAGES
	Gdiplus::GdiplusShutdown(gdiplusToken);
#endif
}

void SavePNG(Gdiplus::Image * image, const wchar_t * filename)
{
#ifdef DRAW_DEBUG_IMAGES
	image->Save(filename, &pngEncoder);
#endif
}