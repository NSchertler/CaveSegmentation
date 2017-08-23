#include "pcplot.h"

#include "GLView.h"
#include <QFileDialog>

struct ParameterSet
{
	float power;
	int algo;
	float scale, size, sizeDerivative;
	float tipPoint, directionTolerance;

	float meanPlausibility;
	float minPlausibility;
};

PCPlot::PCPlot(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	auto fname = QFileDialog::getOpenFileName(this, "Open Evaluation Result", QString(), "*.bin"); //"C:/Users/Nico/Desktop/Evaluation/Evaluation/coarse/result.bin";//
	if (fname.isEmpty())
		return;	

	auto plot = new GLView(this);
	setCentralWidget(plot);
		
	ParameterSet* buffer = new ParameterSet[1024];
	FILE* f = fopen(fname.toStdString().c_str(), "rb");
	while (!feof(f))
	{
		int count = fread(buffer, sizeof(ParameterSet), 1024, f);
		for (int i = 0; i < count; ++i)
		{
			if (buffer[i].algo != 0)
				continue;
			data.push_back(buffer[i].minPlausibility);
			data.push_back(buffer[i].power);
			data.push_back(buffer[i].algo);
			data.push_back(buffer[i].scale);
			data.push_back(buffer[i].size);
			data.push_back(buffer[i].sizeDerivative);
			data.push_back(buffer[i].tipPoint);
			data.push_back(buffer[i].directionTolerance);
		}
	}

	plot->setData(data.data(), data.size() / 8, 8);

	//plot->addAxis("plausibility", 0);
	plot->addAxis("e", 1);
	//plot->addAxis("algo", 2);
	plot->addAxis(QString::fromWCharArray(L"μ<sub>scale</sub>"), 3);
	plot->addAxis(QString::fromWCharArray(L"μ<sub>size</sub>"), 4);
	plot->addAxis(QString::fromWCharArray(L"μ<sub>size'</sub>"), 5);
	plot->addAxis(QString::fromWCharArray(L"θ<sub>tip</sub>"), 6);
	plot->addAxis(QString::fromWCharArray(L"θ<sub>dir</sub>"), 7);

	plot->prepare();
}

PCPlot::~PCPlot()
{

}