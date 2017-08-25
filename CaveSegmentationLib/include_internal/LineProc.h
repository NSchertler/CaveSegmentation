#pragma once

#include "Options.h"

#include <vector>
#include <list>
#include <fstream>

#include "RegularUniformSphereSampling.h"
#include "SphereVisualizer.h"

const double MAX_FLOW_DISTANCE = M_PI / 2 / (SPHERE_SAMPLING_RESOLUTION - 1);
const double LINE_POINT_DISTANCE = 0.1; //geodesic distance

struct PositionGradient
{
	Vector position;
	Vector gradient;

	PositionGradient(){}
	PositionGradient(const Vector& position) : position(position) {}
	PositionGradient(const Vector& position, const Vector& gradient) : position(position), gradient(gradient) {}
};

template<bool CIRCULAR, typename T>
void FindLinkedListNeighbors(std::list<T>& list,
	const typename std::list<T>::iterator it,
	const int steps,
	typename std::list<T>::iterator& prev,
	typename std::list<T>::iterator& next)
{
	prev = it;
	next = it;
	for (int i = 0; i < steps; ++i)
	{
		if (prev == list.begin())
		{
			if (CIRCULAR)
			{
				prev = list.end();
				--prev;
			}
		}
		else
		{
			--prev;
		}

		++next;
		if (next == list.end())
		{
			if (CIRCULAR)
			{
				next = list.begin();
			}
			else
			{
				--next;
			}
		}
	}
}

template<bool CIRCULAR, typename TSphereVisualizer>
void LineFlow(std::list<PositionGradient>& linePoints, const RegularUniformSphereSampling& sphereSampling, const std::vector<std::vector<double>>& potential, const std::vector<std::vector<Vector>>& potentialGradient, double direction, double& optimalAveragePotential, double& optimalLineLength, TSphereVisualizer& visualizer, const std::wstring& baseFilename)
{
	int stopAfterIterations = 20;
	optimalAveragePotential = std::numeric_limits<double>::infinity() * (- direction);

#ifdef WRITE_SPHERE_STATS
	std::ofstream lineTracingStats(baseFilename + std::wstring(L".csv"));
	lineTracingStats.imbue(std::locale(""));
	lineTracingStats << "iteration;averageDistance;maxGradientSquareMagnitude" << std::endl;
#endif

	for (int iteration = 0; iteration < 400; ++iteration)
	{
		if (stopAfterIterations == 0)
			break;
		--stopAfterIterations;

		double maxGradientSquareMagnitude = 0;

		//line flow preparation
		for (auto it = linePoints.begin(); it != linePoints.end(); ++it)
		{
			Vector p = it->position;

			double phi, theta;
			sphereSampling.ParametersFromPoint(p, phi, theta);
		
			Vector g = sphereSampling.AccessInterpolatedContainerData(potentialGradient, phi, theta);						
		
			if (!CIRCULAR)
			{
				std::list<PositionGradient>::iterator prev, next;
				FindLinkedListNeighbors<CIRCULAR>(linePoints, it, 1, prev, next);
				if (it == prev || it == next)
					g = Vector(0, 0, 0); //fix end points
			}

			it->gradient = g;

			//Previous attempts for orthogonal line movement
			/*
			std::list<PositionGradient>::iterator prev, next;
			FindCircularLinkedListNeighbors(linePoints, it, 3, prev, next);

			Vector lineDir = next->position - prev->position;

			//possible movement direction
			Vector moveDir = CGAL::cross_product(p, lineDir);

			//project gradient on movement direction
			double weight = 1 - g * lineDir / sqrt(g.squared_length()) / sqrt(lineDir.squared_length());
			Vector orthogonalPart = (g * moveDir) / moveDir.squared_length() * moveDir;
			Vector parallelPart = (g - orthogonalPart);
			parallelPart = parallelPart - (parallelPart * p) * p;
			//g = g; // orthogonalPart + 0.1 * parallelPart;

			//allow only movements orthogonal to the line
			//g = g - (g * lineDir) / lineDir.squared_length() * lineDir;

			//project gradient on sphere
			//g = g - (g * p) * p;
			*/

			double l = g.squared_length();
			if (l > maxGradientSquareMagnitude)
				maxGradientSquareMagnitude = l;
		}

		double gradientMultiplier = MAX_FLOW_DISTANCE / sqrt(maxGradientSquareMagnitude);


		//actual flow
		for (auto it = linePoints.begin(); it != linePoints.end(); ++it)
		{
			Vector p = it->position + it->gradient * gradientMultiplier * direction;

			Vector oldP = p;

			p = p / sqrt(p.squared_length());

			if (abs(p.x() * p.x() + p.y() * p.y() + p.z() * p.z() - 1) > 0.01)
				std::cout << "Normalization of p failed. Input vector: " << oldP << ", output: " << p << std::endl;


			it->position = p;
		}

		//repair line if necessary
		for (auto it = linePoints.begin(); it != linePoints.end();)
		{
			bool dontIncrement = false;

			Vector p = it->position;

			std::list<PositionGradient>::iterator prev, next;
			FindLinkedListNeighbors<CIRCULAR>(linePoints, it, 1, prev, next);

			if (next != it) //if this is not the last element
			{
				double distanceToNext = acos(it->position * next->position);
				if (distanceToNext > 1.5 * LINE_POINT_DISTANCE)
				{
					Vector meanPosition = it->position + next->position;

					Vector oldMean = meanPosition; //debug

					meanPosition = meanPosition / sqrt(meanPosition.squared_length());

					if (abs(meanPosition.x() * meanPosition.x() + meanPosition.y() * meanPosition.y() + meanPosition.z() * meanPosition.z() - 1) > 0.01)
						std::cout << "Normalization of mean failed. Input vector: " << oldMean << ", output: " << meanPosition << std::endl;

					linePoints.insert(next, PositionGradient(meanPosition));
				}

				if (prev != it) //if this is not the first or last element
				{
					double distancePrevToNext = acos(prev->position * next->position);
					if (distanceToNext < 0.25 * LINE_POINT_DISTANCE || distancePrevToNext < 0.5 * LINE_POINT_DISTANCE)
					{
						it = linePoints.erase(it);
						dontIncrement = true;
					}
				}
			}

			if (!dontIncrement)
				++it;
		}

		//calculate average potential
		double lineLength = 0;
		double averagePotential = 0;

		for (auto it = linePoints.begin(); it != linePoints.end(); ++it)
		{
			Vector p = it->position;

			if (abs(p.squared_length() - 1) > 0.01)
				std::cout << "Point on line has non-unit length:" << p << std::endl;

			double distance = sphereSampling.AccessInterpolatedContainerData(potential, p);

			std::list<PositionGradient>::iterator prev, next;
			FindLinkedListNeighbors<CIRCULAR>(linePoints, it, 1, prev, next);

			double distanceToPrev = acos(std::min(1.0, std::max(-1.0, it->position * prev->position)));
			double distanceToNext = acos(std::min(1.0, std::max(-1.0, it->position * next->position)));
			double vertexDiameter = (distanceToPrev + distanceToNext) / 2;
			lineLength += vertexDiameter;
			averagePotential = averagePotential + vertexDiameter / lineLength * (distance - averagePotential);
		}

		if (averagePotential * direction > optimalAveragePotential * direction)	//found a local extremum...
		{
			stopAfterIterations = 20;				//check 20 more iterations
			optimalAveragePotential = averagePotential;
			optimalLineLength = lineLength;
		}

#ifdef WRITE_SPHERE_STATS
		lineTracingStats << iteration << ";" << averagePotential << ";" << maxGradientSquareMagnitude << std::endl;
#endif

		//draw separate image
		TSphereVisualizer stateVis(visualizer);				
		for (auto& p : linePoints)
		{
			double phi, theta;
			sphereSampling.ParametersFromPoint(p.position, phi, theta);

			stateVis.FillCircle(theta, phi, 3, SphereVisualizer::FLOW_OUTLINE_COLOR);
			stateVis.FillCircle(theta, phi, 2, SphereVisualizer::FLOW_COLOR);
		}
		std::wstring filename = baseFilename + L"_" + std::to_wstring(iteration) + L".png";
		stateVis.Save(filename);
	}

#ifdef WRITE_SPHERE_STATS
	lineTracingStats.close();
#endif
}


struct MeanAverageWorkspace
{
	struct TableEntry
	{
		double accumulatedValue;
		int predecessor; //sequence last index
	};

	TableEntry* data;
	int dataCount;
	std::vector<std::pair<int, int>> columnBounds; //lower bound inclusive, upper bound exlusive

	MeanAverageWorkspace() : dataCount(0), data(nullptr) {}

	~MeanAverageWorkspace()
	{
		if (data)
			delete[] data;
	}

	void Initialize(int sampleCount, int maxSubsetSize)
	{
		this->sampleCount = sampleCount;
		int requiredDataCount = sampleCount * maxSubsetSize;
		if (dataCount < requiredDataCount)
		{
			if (data)
				delete[] data;
			data = new TableEntry[requiredDataCount];
			dataCount = requiredDataCount;
		}
		if(!columnBounds.empty())
			memset(&columnBounds[0], 0, sizeof(std::pair<int, int>) * columnBounds.size()); //reset column bounds
		columnBounds.resize(sampleCount, std::pair<int, int>(0, 0));
	}

	TableEntry& Entry(int subsetSize, int sampleIndex)
	{
		auto& bounds = columnBounds[sampleIndex];
		if (subsetSize < bounds.first || bounds.first == 0)
			bounds.first = subsetSize;
		if (subsetSize + 1 > bounds.second)
			bounds.second = subsetSize + 1;
		return data[sampleIndex + (subsetSize - 1) * sampleCount];
	}

	bool HasEntry(int subsetSize, int sampleIndex)
	{
		auto& bounds = columnBounds[sampleIndex];
		return subsetSize >= bounds.first && subsetSize < bounds.second;
	}

	void ColumnBounds(int sampleIndex, int& minInclusive, int& maxExclusive)
	{
		auto& bounds = columnBounds[sampleIndex];
		minInclusive = bounds.first;
		maxExclusive = bounds.second;
	}

private:
	int sampleCount;
};

/*
 Assumes line to be a sampling of a sphere-surrounding line. Calculates

     arg min 1 / |R| * Sum{r \in R} (line[r].value)
	    R

		 s.t minAngularDistance <= line[R[i + 1]].angle - line[R[i]].angle <= maxAngularDistance \forall i \in {0, |R| - 2}
		     minAngularDistance <= line[R[0]].angle - line[R[|R| - 1]].angle + 2 Pi <= maxAngularDistance

 T - must have double value and double angle
 line - sorted wrt. angle
*/
template <typename T>
void FindOptimalMeanAverage(const std::vector<T>& line, double minAngularDistance, double maxAngularDistance, MeanAverageWorkspace& workspace, std::vector<int>& result)
{
	/*
	This DP algorithm successively calculates the following table:
	
	k \ idx  |  0  |  1  |  2  | ... | ... | ... |  n
	---------+-----+-----+-----+-----+-----+-----+-----
	    1    |  
		2    |
		3    |
	  k_max  |
	                      | <-  accept  -> |

	where the entries denote the minimum sum of a sequence with k entries ending at idx.
	The optimal solution is the entry in the accept-range with the minimum mean (= value / k).
	*/

	int n = (int)line.size();

	double globalOptimalMean = std::numeric_limits<double>::infinity();

	for (int firstPickedSample = 0; firstPickedSample < n; ++firstPickedSample)
	{
		if (line.at(firstPickedSample).angle - line.at(0).angle > maxAngularDistance)
			break;

		workspace.Initialize(n, (int)floor(2 * M_PI / minAngularDistance));

		double acceptMin = line.at(firstPickedSample).angle + 2 * M_PI - maxAngularDistance;
		double acceptMax = line.at(firstPickedSample).angle + 2 * M_PI - minAngularDistance;

		double optimalMeanSoFar = std::numeric_limits<double>::infinity();
		int optimalMeanIdx = -1;
		int optimalMeanK = -1;

		//Initialize the table
		auto& initialEntry = workspace.Entry(1, firstPickedSample);
		initialEntry.accumulatedValue = line.at(firstPickedSample).value;
		initialEntry.predecessor = -1;

		//Start DP
		for (int currentIdx = firstPickedSample; currentIdx < n; ++currentIdx)
		{
			//the next sample must be in this range
			double minNextAngle = line.at(currentIdx).angle + minAngularDistance;
			double maxNextAngle = std::min(line.at(currentIdx).angle + maxAngularDistance, acceptMax);

			if (maxNextAngle < minNextAngle)
				break; //we cannot find a new sample that falls in the accept range

			int kLower, kUpper;
			workspace.ColumnBounds(currentIdx, kLower, kUpper);

			for (int searchIdx = currentIdx + 1; searchIdx < n; ++searchIdx)
			{
				double angle = line.at(searchIdx).angle;
				if (angle < minNextAngle)
					continue;
				if (angle > maxNextAngle)
					break;

				double newSampleValue = line.at(searchIdx).value;

				for (int currentK = kLower; currentK < kUpper; ++currentK)
				{
					double newAccumulatedValue = workspace.Entry(currentK, currentIdx).accumulatedValue + newSampleValue;
					bool overwrite = !workspace.HasEntry(currentK + 1, searchIdx);
					auto& entry = workspace.Entry(currentK + 1, searchIdx);
					if (!overwrite)
						overwrite = newAccumulatedValue < entry.accumulatedValue;
					if (overwrite)
					{
						entry.accumulatedValue = newAccumulatedValue;
						entry.predecessor = currentIdx;

						//if we are in the accept range
						if (angle >= acceptMin)
						{
							//check if the new mean is better than the current best mean
							double mean = newAccumulatedValue / (currentK + 1);
							if (mean < optimalMeanSoFar)
							{
								optimalMeanSoFar = mean;
								optimalMeanIdx = searchIdx;
								optimalMeanK = currentK + 1;
							}
						}
					}
				}
			}
		}
		if (optimalMeanSoFar < globalOptimalMean)
		{
			result.clear();
			while (optimalMeanIdx >= 0)
			{
				result.push_back(optimalMeanIdx);
				optimalMeanIdx = workspace.Entry(optimalMeanK, optimalMeanIdx).predecessor;
				--optimalMeanK;
			}
			globalOptimalMean = optimalMeanSoFar;
		}
	}	
}