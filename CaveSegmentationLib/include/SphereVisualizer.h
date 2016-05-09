#pragma once

#include "ImageProc.h"
#include "CGALCommon.h"
#include "RegularUniformSphereSampling.h"

#include <vector>

// Represents a class that creates a 2D visualization (parameter space) of a sphere.
class SphereVisualizer
{
	const int imWidth = 1280;
	const int imHeight = imWidth / 2;

	std::wstring outputDirectoryW;
public:
	static const Gdiplus::Color SAMPLE_COLOR;
	static const Gdiplus::Color VORONOI_COLOR;
	static const Gdiplus::Color FLOW_COLOR;
	static const Gdiplus::Color FLOW_OUTLINE_COLOR;
	static const Gdiplus::Color SEPARATING_CIRCLE_COLOR;
	static const Gdiplus::Color SEPARATING_LINE_COLOR;

	SphereVisualizer();
	SphereVisualizer(const SphereVisualizer&);
	~SphereVisualizer();

	void SetOutputDirectory(const std::wstring&);

	void FillRect(int x, int y, int w, int h, const Gdiplus::Color& color);
	void FillCircle(int x, int y, int radius, const Gdiplus::Color& color);

	void DrawRect(int x, int y, int w, int h, const Gdiplus::Color& color);
	void DrawGradientField(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<Vector>>& gradient);

	void Save(const std::wstring& filename);

private:
#ifdef DRAW_DEBUG_IMAGES
	Gdiplus::Bitmap* bitmap;
	Gdiplus::Graphics* graphics;
#endif
};