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

	float segmentationPlausability() const
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

		return (float)(1 - unplausability / data.meshVertices().size());
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
	float power;
	CaveData::Algorithm algo;
	float scale, size, sizeDerivative;
	float tipPoint, directionTolerance;

	float meanPlausibility = 0;
	float minPlausibility = 0;
};

std::ostream& operator<<(std::ostream& os, CaveData::Algorithm a)
{
	switch (a)
	{
	case CaveData::Max:
		return os << "Max";
	case CaveData::Smooth:
		return os << "Smooth";
	case CaveData::Advect:
		return os << "Advect";
	}
}

std::ostream& operator<<(std::ostream& os, const ParameterSet& p)
{
	os << "Mean Plausibility: " << (p.meanPlausibility * 100.0) << " %" << std::endl;
	os << "Min Plausibility: " << (p.minPlausibility * 100.0) << " %" << std::endl;
	os << "Distance Power: " << p.power << std::endl;
	os << "Scale Algorithm: " << p.algo << std::endl;
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
	std::cout.imbue(std::locale("en-US"));

	std::vector<CaveInfo*> caves;
	for (int i = 1; i < argc; ++i)
	{
		std::cout << "Loading \"" << argv[i] << "\".." << std::endl;
		caves.push_back(new CaveInfo(std::string(argv[i])));		
	}

	std::vector<float> plausabilities(caves.size());

	ParameterRange<float> powerRange;
	ParameterRange<float> scaleRange;
	ParameterRange<float> sizeRange;
	ParameterRange<float> sizeDerivativeRange;
	ParameterRange<float> tipPointRange;
	ParameterRange<float> directionToleranceRange;

	ParameterRange<float>* configurableRanges[] = { &powerRange, &scaleRange, &sizeRange, &sizeDerivativeRange, &tipPointRange, &directionToleranceRange };
	std::ifstream config("config.txt");

	std::string line;
	std::getline(config, line);
	std::istringstream ss(line);
	std::vector<CaveData::Algorithm> algos;
	while (!ss.eof())
	{
		int algo;
		ss >> algo;
		if (ss.fail())
			break;
		auto realAlgo = static_cast<CaveData::Algorithm>(algo);
		algos.push_back(realAlgo);
		std::cout << "Scheduling algorithm for evaluation: " << realAlgo << std::endl;
	}
	
	for (int i = 0; i < 6; ++i)
	{
		std::getline(config, line);
		std::istringstream ss(line);
		ss >> configurableRanges[i]->lower >> configurableRanges[i]->upper >> configurableRanges[i]->steps;
	}

	size_t totalIterations = 
		(powerRange.steps + 1)
		* (algos.size())
		* (scaleRange.steps + 1)
		* (sizeRange.steps + 1)
		* (sizeDerivativeRange.steps + 1)
		* (tipPointRange.steps + 1)
		* (directionToleranceRange.steps + 1);
	std::cout << "Evaluating a total of " << totalIterations << " samples." << std::endl;
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

	FILE* binFile = fopen("result.bin", "wb");
	if (!binFile)
	{
		std::cerr << "Could not open binary file." << std::endl;
		csvFile.close();
		return 2;
	}

	std::ios oldState(nullptr);
	oldState.copyfmt(std::cout);
	std::cout << std::fixed;
	std::cout.precision(1);

	csvFile.imbue(std::locale("en-US"));
	csvFile << "Distance Power;Scale Algorithm;Scale Kernel;Size Kernel;Size Derivative Kernel;Curvature Tipping Point;Direction Tolerance;Mean Plausability;Min Plausability;";
	for (int i = 0; i < caves.size(); ++i)
		csvFile << "Plausability Cave " << i << ";";
	csvFile << std::endl;
	
	Timer<> timer;
	for (float _power : powerRange)
	{
		params.power = _power;

		for (int i = 0; i < caves.size(); ++i)
			caves.at(i)->data.CalculateDistances(params.power);

		for (auto _algo : algos)
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

						double percentage = (double)currentIterations / totalIterations;
						std::cout << "\rStatus: " << (100.0 * percentage) << " %, estimated time to finish: " << timeString(timer.value() / percentage - timer.value()) << "        ";

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
									float p = caves.at(i)->segmentationPlausability();

									plausabilities.at(i) = p;
									int c = caves.at(i)->data.meshVertices().size();
									countVertices += c;
									params.meanPlausibility += (float)c / countVertices * (p - params.meanPlausibility);
									params.minPlausibility = std::min(params.minPlausibility, p);
								}

								csvFile << params.power << ";" << (int)params.algo << ";" << params.scale << ";" << params.size << ";" << params.sizeDerivative << ";" << params.tipPoint << ";" << params.directionTolerance << ";" << params.meanPlausibility << ";" << params.minPlausibility << "; ";
								for (float p : plausabilities)
									csvFile << p << "; ";
								csvFile << std::endl;
								fwrite(&params, sizeof(ParameterSet), 1, binFile);

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

	std::cout.copyfmt(oldState);

	csvFile.close();
	fclose(binFile);

	std::cout << std::endl;	

	std::cout << std::endl << "Best parameters for minimal plausibility:" << std::endl << bestMinParameters;
	std::cout << std::endl << "Best parameters for average plausibility:" << std::endl << bestAverageParameters;		

    return 0;
}

