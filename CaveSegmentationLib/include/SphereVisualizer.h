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

	const std::wstring& outputDirectoryW;
public:
	static const Gdiplus::Color SAMPLE_COLOR;
	static const Gdiplus::Color VORONOI_COLOR;
	static const Gdiplus::Color FLOW_COLOR;
	static const Gdiplus::Color FLOW_OUTLINE_COLOR;
	static const Gdiplus::Color SEPARATING_CIRCLE_COLOR;
	static const Gdiplus::Color SEPARATING_LINE_COLOR;

	SphereVisualizer(const std::wstring& outputDirectory);
	SphereVisualizer(const SphereVisualizer&);
	~SphereVisualizer();

	void FillRect(double theta, double phi, double sizeTheta, double sizePhi, const Gdiplus::Color& color);
	void FillCircle(double theta, double phi, int radiusPixels, const Gdiplus::Color& color);

	void DrawRect(double theta, double phi, double sizeTheta, double sizePhi, const Gdiplus::Color& color);
	void DrawGradientField(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<Vector>>& gradient);

	void Save(const std::wstring& filename);

private:
	Gdiplus::Bitmap* bitmap;
	Gdiplus::Graphics* graphics;
};

class VoidSphereVisualizer
{
public:
	VoidSphereVisualizer(const std::wstring& outputDirectory);
	VoidSphereVisualizer(const VoidSphereVisualizer&);

	void FillRect(double theta, double phi, double sizeTheta, double sizePhi, const Gdiplus::Color& color);
	void FillCircle(double theta, double phi, int radiusPixels, const Gdiplus::Color& color);

	void DrawRect(double theta, double phi, double sizeTheta, double sizePhi, const Gdiplus::Color& color);
	void DrawGradientField(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<Vector>>& gradient);

	void Save(const std::wstring& filename);
};