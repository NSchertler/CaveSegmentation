#pragma once

#include <iostream>

#include "../CaveData.h"
#include "energies.h"

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/inference/external/qpbo.hxx>

class CurvatureBasedQPBO
{
	static constexpr double ENTRANCE_MIN_DISTANCE_TO_END = 20.0; //entrances must have at least this distance from the nearest cave end

	static void propagateDistance(const CaveData& data, int vertex, double currentDistance, std::vector<double>& distancesFromEnd)
	{
		if (currentDistance < distancesFromEnd.at(vertex) && currentDistance < ENTRANCE_MIN_DISTANCE_TO_END)
		{
			distancesFromEnd.at(vertex) = currentDistance;
			for (int neighbor : data.adjacency.at(vertex))
			{
				double distance = (data.skeleton->vertices.at(vertex).position - data.skeleton->vertices.at(neighbor).position).norm();
				propagateDistance(data, neighbor, currentDistance + distance, distancesFromEnd);
			}
		}
	}

public:
	static void FindChambers(const CaveData& data, std::vector<int>& segmentation)
	{
		/*std::cout << "Calculating distances from ends" << std::endl;

		std::vector<double> distancesFromEnd(data.skeleton->vertices.size(), std::numeric_limits<double>::infinity());
		for (int v = 0; v < data.skeleton->vertices.size(); ++v)
			if (data.adjacency.at(v).size() == 1)
			{
				propagateDistance(data, v, 0, distancesFromEnd);
			}			*/

		opengm::DiscreteSpace<> labelSpace;
		typedef opengm::GraphicalModel<double, opengm::Adder> Model;

		//Label 0: Chamber
		//Label 1: Passage
		for (unsigned int i = 0; i < data.skeleton->vertices.size(); ++i)
			labelSpace.addVariable(2);

		Model gm(labelSpace);

		int nonsubmodular = 0;

		//pairwise factors
		const size_t functionShapePairwise[] = { 2, 2 };
		for (int iEdge = 0; iEdge < data.skeleton->edges.size(); ++iEdge)
		{
			auto& edge = data.skeleton->edges.at(iEdge);
			auto v1 = data.skeleton->vertices.at(edge.first);
			auto v2 = data.skeleton->vertices.at(edge.second);

			double derivative = data.caveSizeDerivativesPerEdge.at(iEdge);
			double curvature = data.caveSizeCurvaturesPerEdge.at(iEdge);

			double entranceProbability = ::entranceProbability(curvature * 0.5 * (data.caveScale.at(edge.first) + data.caveScale.at(edge.second)));
			double directionProbability = ::directionProbability(derivative);

			//only allow entrances that have a minimum distance to the cave end
			/*if (distancesFromEnd.at(edge.first) < ENTRANCE_MIN_DISTANCE_TO_END && distancesFromEnd.at(edge.second) < ENTRANCE_MIN_DISTANCE_TO_END)
				entranceProbability = 0;*/

			auto pairwiseFunction = opengm::ExplicitFunction<double>(functionShapePairwise, functionShapePairwise + 2);			
			pairwiseFunction(0, 0) = -log(1 - entranceProbability);
			pairwiseFunction(1, 1) = -log(1 - entranceProbability);
			int nodes[2];
			if (edge.first < edge.second)
			{
				pairwiseFunction(1, 0) = -log(entranceProbability * directionProbability);
				pairwiseFunction(0, 1) = -log(entranceProbability * (1 - directionProbability));
				nodes[0] = edge.first;
				nodes[1] = edge.second;
			}
			else
			{
				pairwiseFunction(0, 1) = -log(entranceProbability * directionProbability);
				pairwiseFunction(1, 0) = -log(entranceProbability * (1 - directionProbability));
				nodes[1] = edge.first;
				nodes[0] = edge.second;
			}

			//std::cout << pairwiseFunction(0, 0) << "; " << pairwiseFunction(1, 0) << ", " << pairwiseFunction(0, 1) << std::endl;

			gm.addFactor(gm.addFunction(pairwiseFunction), nodes, nodes + 2);
			if (pairwiseFunction(0, 0) + pairwiseFunction(1, 1) > pairwiseFunction(0, 1) + pairwiseFunction(1, 0))
				++nonsubmodular;
		}

#ifndef NON_VERBOSE
		std::cout << "Solving minimization problem (" << gm.numberOfFactors() << " factors, " << nonsubmodular << " non-submodular, " << gm.numberOfVariables() << " variables) ..." << std::endl;
#endif

		typedef opengm::external::QPBO<Model> Optimizer;

		Optimizer::Parameter param;
		param.strongPersistency_ = false;
		param.useProbeing_ = false;
		param.useImproveing_ = false;

		Optimizer optimizer(gm, param);
		optimizer.infer();

#ifndef NON_VERBOSE
		std::cout << "Generating output..." << std::endl;
#endif

		std::vector<size_t> argmin;
		optimizer.arg(argmin);
		segmentation.resize(argmin.size());
		for (int i = 0; i < argmin.size(); ++i)
			segmentation[i] = (argmin[i] == 0 ? 0 : -1);
	}
};