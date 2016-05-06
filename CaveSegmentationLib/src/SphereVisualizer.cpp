#include "SphereVisualizer.h"

#include "Options.h"

using namespace Gdiplus;

const Gdiplus::Color SphereVisualizer::SAMPLE_COLOR = Gdiplus::Color(0, 0, 255);
const Gdiplus::Color SphereVisualizer::VORONOI_COLOR = Gdiplus::Color(0, 150, 0);
const Gdiplus::Color SphereVisualizer::FLOW_COLOR = Gdiplus::Color(255, 255, 0);
const Gdiplus::Color SphereVisualizer::FLOW_OUTLINE_COLOR = Gdiplus::Color(200, 100, 0);
const Gdiplus::Color SphereVisualizer::SEPARATING_CIRCLE_COLOR = Gdiplus::Color(127, 0, 0);
const Gdiplus::Color SphereVisualizer::SEPARATING_LINE_COLOR = Gdiplus::Color(255, 0, 0);

SphereVisualizer::SphereVisualizer()
{
#ifdef DRAW_DEBUG_IMAGES
	bitmap = new Bitmap(imWidth, imHeight, PixelFormat24bppRGB);
	graphics = new Graphics(bitmap);
	graphics->Clear(Gdiplus::Color(0, 0, 0, 0));
#endif
}

SphereVisualizer::SphereVisualizer(const SphereVisualizer& clone)
{
#ifdef DRAW_DEBUG_IMAGES
	bitmap = new Bitmap(imWidth, imHeight, PixelFormat24bppRGB);
	graphics = new Graphics(bitmap);
	graphics->DrawImage(clone.bitmap, 0, 0, imWidth, imHeight);
#endif
}

SphereVisualizer::~SphereVisualizer()
{
#ifdef DRAW_DEBUG_IMAGES
	delete graphics;
	delete bitmap;
#endif
}

void SphereVisualizer::FillRect(int x, int y, int w, int h, const Gdiplus::Color& color)
{
#ifdef DRAW_DEBUG_IMAGES
	SolidBrush brush(color);
	graphics->FillRectangle(&brush, x, y, w, h);
#endif
}

void SphereVisualizer::FillCircle(int x, int y, int radius, const Gdiplus::Color & color)
{
#ifdef DRAW_DEBUG_IMAGES
	SolidBrush brush(color);
	graphics->FillEllipse(&brush, (int)(x - radius), (int)(y - radius), 2 * radius, 2 * radius);
#endif
}

void SphereVisualizer::DrawRect(int x, int y, int w, int h, const Gdiplus::Color & color)
{
#ifdef DRAW_DEBUG_IMAGES
	SolidBrush brush(color);
	Pen pen(&brush);
	graphics->DrawRectangle(&pen, x, y, w, h);
#endif
}

void SphereVisualizer::DrawGradientField(const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<Vector>>& gradientField)
{
#ifdef DRAW_DEBUG_IMAGES
	for (auto it = sphereSampling.begin(); it != sphereSampling.end(); ++it)
	{
		double theta, phi, wTheta, wPhi;
		it.GetParameterSpaceRect(theta, phi, wTheta, wPhi);

		Gdiplus::SolidBrush normalBrush(Gdiplus::Color(0, 255, 0));
		Gdiplus::SolidBrush longBrush(Gdiplus::Color(0, 128, 128));
		Gdiplus::Pen normalPen(&normalBrush);
		Gdiplus::Pen longPen(&longBrush);
		const Vector& gradient = sphereSampling.AccessContainerData(gradientField, it);

		//transform positional gradient to parameter space gradient
		//with inverse Jacobian
		// | a  b  c | --> phi
		// | d  e  f | --> theta
		double cp = cos(phi);
		double sp = sin(phi);
		double ct = cos(theta);
		double st = sin(theta);

		double cp2 = cp * cp;
		double ct2 = ct * ct;
		double sp2 = sp * sp;
		double st2 = st * st;
		double sp3 = sp2 * sp;

		double denom = -cp2 * ct2 * sp - ct2 * sp2 - cp2 * sp * st2 - sp3 * st2;
		double a = -cp * sp * st / denom;
		double b = -cp * ct * sp / denom;
		double c = (ct2 * sp2 + sp2 * st2) / denom;
		double d = (-cp2 * ct - ct * sp2) / denom;
		double e = (cp2 * st + sp2 * st) / denom;
		double f = 0;

		/*double a = cp * st;
		double b = cp * ct;
		double c = -sp;
		double d = ct * sp;
		double e = -sp*st;
		double f = 0;*/

		double gradientPhi = (a * gradient.x() + b * gradient.y() + c * gradient.z()) / 100;
		double gradientTheta = (d * gradient.x() + e * gradient.y() + f * gradient.z()) / 100;

		theta *= imWidth / (2 * M_PI);
		wTheta *= imWidth / (2 * M_PI);
		phi *= imHeight / M_PI;
		wPhi *= imHeight / M_PI;

		gradientPhi *= imHeight / M_PI;
		gradientTheta *= imWidth / (2 * M_PI);

		Gdiplus::Pen* pen = &normalPen;

		double l = sqrt(gradientPhi * gradientPhi + gradientTheta * gradientTheta);
		if (l > 30)
		{
			gradientPhi *= 30.0 / l;
			gradientTheta *= 30.0 / l;
			pen = &longPen;
		}

		graphics->DrawLine(pen, (int)(theta + wTheta / 2), (int)(phi + wPhi / 2), (int)(theta + wTheta / 2 + gradientTheta), (int)(phi + wPhi / 2 + gradientPhi));
	}
#endif
}

void SphereVisualizer::Save(const std::wstring& filename)
{
#ifdef DRAW_DEBUG_IMAGES
	SavePNG(bitmap, filename.c_str());
#endif
}
