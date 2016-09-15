#include <iostream>

#include "CaveData.h"
#include "ChamberAnalyzation/CurvatureBasedQPBO.h"

#include <boost/filesystem.hpp>

struct CaveInfo
{
	CaveInfo(std::string dataDirectory)
	{
		const std::string offFile = dataDirectory + "/model.off";
		const std::string skeletonFile = dataDirectory + "/model.skel";

		data.LoadMesh(offFile);

		CurveSkeleton* skeleton = LoadCurveSkeleton(skeletonFile.c_str());
		data.SetSkeleton(skeleton);

		chamberProbability.resize(data.meshVertices().size(), 0.5);
		segmentation.resize(data.meshVertices().size());

		boost::filesystem::path p(dataDirectory + "/segmentations");
		int nSegs = 0;
		std::vector<int> manualSeg(data.meshVertices().size());
		for (auto it = boost::filesystem::directory_iterator(p); it != boost::filesystem::directory_iterator(); ++it)
		{
			boost::filesystem::path file = *it;
			if (file.extension() == ".caveseg")
			{
				++nSegs;				
				FILE* manualSegFile = fopen(file.string().c_str(), "rb");
				fread(manualSeg.data(), sizeof(int), data.meshVertices().size(), manualSegFile);
				fclose(manualSegFile);

				//in manual cave segmentation:
				/*enum SegmentationType
				{
				Erase = 0,
				Chamber = 1,
				Passage = 2,
				};*/

				for (int i = 0; i < data.meshVertices().size(); ++i)
				{
					if (manualSeg.at(i) == 1)
						chamberProbability.at(i) += (1.0 - chamberProbability.at(i)) / nSegs;
					if (manualSeg.at(i) == 2)
						chamberProbability.at(i) += (0.0 - chamberProbability.at(i)) / nSegs;
				}
			}
		}
	}

	void segment()
	{
		CurvatureBasedQPBO::FindChambers(data, segmentation);
	}

	double segmentationPlausability() const
	{
		//seg result:
		//0 -> chamber
		//-1 -> passage

		double unplausability = 0;
		for (int i = 0; i < data.skeleton->vertices.size(); ++i)
		{
			auto& v = data.skeleton->vertices.at(i);
			for (int meshVertex : v.correspondingOriginalVertices)
			{
				if (chamberProbability.at(meshVertex) < 0.5 && segmentation.at(i) == 0)
					unplausability += (0.5 - chamberProbability.at(meshVertex)) * 2;
				if (chamberProbability.at(meshVertex) > 0.5 && segmentation.at(i) < 0)
					unplausability += (chamberProbability.at(meshVertex) - 0.5) * 2;
			}
		}

		return 1 - unplausability / data.meshVertices().size();
	}

	CaveData data;
	std::vector<double> chamberProbability;
	std::vector<int> segmentation;
};

template <typename T>
struct ParameterRange
{
	ParameterRange()
	{ }

	ParameterRange(T lower, T upper, int steps)
		: lower(lower), upper(upper), steps(steps)
	{ }

	struct Iterator
	{
		Iterator(const ParameterRange& range, int i)
			: range(range), i(i)
		{ }

		Iterator& operator++() { ++i; return *this; }
		bool operator!=(const Iterator& other) const { return i != other.i; }
		T operator*() const { return static_cast<T>(range.lower + i * (range.upper - range.lower) / range.steps); }

		double percentage() const { return i / (range.steps + 1); }

		const ParameterRange& range;
		int i;
	};

	Iterator begin() { return Iterator(*this, 0); }
	Iterator end() { return Iterator(*this, steps + 1); }

	T lower, upper;
	int steps;
};

struct ParameterSet
{
	double meanPlausibility = 0;
	double minPlausibility = 0;

	double power;
	CaveData::Algorithm algo;
	double scale, size, sizeDerivative;
	double tipPoint, directionTolerance;
};

std::ostream& operator<<(std::ostream& os, const ParameterSet& p)
{
	os << "Mean Plausibility: " << (p.meanPlausibility * 100.0) << " %" << std::endl;
	os << "Min Plausibility: " << (p.minPlausibility * 100.0) << " %" << std::endl;
	os << "Distance Power: " << p.power << std::endl;
	os << "Scale Algorithm: " << (p.algo == CaveData::Max ? "Max" : (p.algo == CaveData::Smooth ? "Smooth" : "Advect")) << std::endl;
	os << "Scale Kernel: " << p.scale << std::endl;
	os << "Size Kernel: " << p.size << std::endl;
	os << "Size Derivative Kernel: " << p.sizeDerivative << std::endl;
	os << "Curvature Tipping Point: " << p.tipPoint << std::endl;
	os << "Direction Tolerance: " << p.directionTolerance << std::endl;

	return os;
}

template <typename TimeT = std::chrono::milliseconds> class Timer {
public:
	Timer() {
		start = std::chrono::system_clock::now();
	}

	size_t value() const {
		auto now = std::chrono::system_clock::now();
		auto duration = std::chrono::duration_cast<TimeT>(now - start);
		return (size_t)duration.count();
	}

	size_t reset() {
		auto now = std::chrono::system_clock::now();
		auto duration = std::chrono::duration_cast<TimeT>(now - start);
		start = now;
		return (size_t)duration.count();
	}
private:
	std::chrono::system_clock::time_point start;
};

inline std::string timeString(double time, bool precise = false) {
	if (std::isnan(time) || std::isinf(time))
		return "inf";

	std::string suffix = "ms";
	if (time > 1000) {
		time /= 1000; suffix = "s";
		if (time > 60) {
			time /= 60; suffix = "m";
			if (time > 60) {
				time /= 60; suffix = "h";
				if (time > 24) {
					time /= 24; suffix = "d";
				}
			}
		}
	}

	std::ostringstream os;
	os << std::setprecision(precise ? 4 : 1)
		<< std::fixed << time << suffix;

	return os.str();
}

int main(int argc, char* argv[])
{
	std::vector<CaveInfo*> caves;
	for (int i = 1; i < argc; ++i)
	{
		std::cout << "Loading \"" << argv[i] << "\".." << std::endl;
		caves.push_back(new CaveInfo(std::string(argv[i])));
	}

	std::vector<double> plausabilities(caves.size());

	ParameterRange<double> powerRange;
	ParameterRange<CaveData::Algorithm> algorithmRange(CaveData::Algorithm::Max, CaveData::Algorithm::Advect, 1);
	ParameterRange<double> scaleRange;
	ParameterRange<double> sizeRange;
	ParameterRange<double> sizeDerivativeRange;
	ParameterRange<double> tipPointRange;
	ParameterRange<double> directionToleranceRange;

	ParameterRange<double>* configurableRanges[] = { &powerRange, &scaleRange, &sizeRange, &sizeDerivativeRange, &tipPointRange, &directionToleranceRange };
	std::ifstream config("config.txt");
	std::string line;
	for (int i = 0; i < 6; ++i)
	{
		std::getline(config, line);
		std::istringstream ss(line);
		ss >> configurableRanges[i]->lower >> configurableRanges[i]->upper >> configurableRanges[i]->steps;
	}

	size_t totalIterations = 
		(powerRange.steps + 1)
		* (algorithmRange.steps + 1)
		* (scaleRange.steps + 1)
		* (sizeRange.steps + 1)
		* (sizeDerivativeRange.steps + 1)
		* (tipPointRange.steps + 1)
		* (directionToleranceRange.steps + 1);
	size_t currentIterations = 0;

	ParameterSet bestAverageParameters;
	ParameterSet bestMinParameters;
	ParameterSet params;

	std::ofstream csvFile;
	csvFile.open("result.csv");
	if (!csvFile.good())
	{
		std::cerr << "Could not open csv file." << std::endl;
		return 1;
	}

	
	Timer<> timer;
	for (double _power : powerRange)
	{
		params.power = _power;

#pragma omp parallel for
		for (int i = 0; i < caves.size(); ++i)
			caves.at(i)->data.CalculateDistances(params.power);

		for (auto _algo : algorithmRange)
		{
			params.algo = _algo;
			for (auto _scale : scaleRange)
			{
				params.scale = _scale;
				for (auto _size : sizeRange)
				{
					params.size = _size;
					for (auto _sizeDerivative : sizeDerivativeRange)
					{
						params.sizeDerivative = _sizeDerivative;

#pragma omp parallel for
						for (int i = 0; i < caves.size(); ++i)
						{
							caves.at(i)->data.CAVE_SCALE_ALGORITHM = params.algo;
							caves.at(i)->data.CAVE_SCALE_KERNEL_FACTOR = params.scale;
							caves.at(i)->data.CAVE_SIZE_KERNEL_FACTOR = params.size;
							caves.at(i)->data.CAVE_SIZE_DERIVATIVE_KERNEL_FACTOR = params.sizeDerivative;

							caves.at(i)->data.SmoothAndDeriveDistances();
						}

						for (auto _tipPoint : tipPointRange)
						{
							params.tipPoint = _tipPoint;
							double percentage = (double)currentIterations / totalIterations;
							std::cout << "\rStatus: " << (100.0 * percentage) << " %, estimated time to finish: " << timeString(timer.value() / percentage - timer.value()) << "        ";

							for (auto _directionTolerance : directionToleranceRange)
							{
								params.directionTolerance = _directionTolerance;

								Energies::CURVATURE_TIP_POINT = params.tipPoint;
								Energies::DIRECTION_TOLERANCE = params.directionTolerance;

								params.meanPlausibility = 0.0;
								params.minPlausibility = 1.0;
								int countVertices = 0;
								for (int i = 0; i < caves.size(); ++i)
								{
									caves.at(i)->segment();
									double p = caves.at(i)->segmentationPlausability();

									plausabilities.at(i) = p;
									int c = caves.at(i)->data.meshVertices().size();
									countVertices += c;
									params.meanPlausibility += (double)c / countVertices * (p - params.meanPlausibility);
									params.minPlausibility = std::min(params.minPlausibility, p);
								}

								csvFile << params.tipPoint << ";" << params.directionTolerance << ";" << params.meanPlausibility << ";" << params.minPlausibility << "; ";
								for (double p : plausabilities)
									csvFile << p << "; ";
								csvFile << std::endl;

								if (params.meanPlausibility > bestAverageParameters.meanPlausibility)
									bestAverageParameters = params;
								if (params.minPlausibility > bestMinParameters.minPlausibility)
									bestMinParameters = params;

								++currentIterations;
							}
						}
					}
				}
			}
			csvFile.flush();
		}
	}

	std::cout << std::endl;

	std::cout << std::endl << "Best parameters for minimal plausibility:" << std::endl << bestMinParameters;
	std::cout << std::endl << "Best parameters for average plausibility:" << std::endl << bestAverageParameters;	

    return 0;
}

