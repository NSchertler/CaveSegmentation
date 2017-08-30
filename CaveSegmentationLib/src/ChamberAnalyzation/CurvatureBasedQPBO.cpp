#include "ChamberAnalyzation/CurvatureBasedQPBO.h"

#include "ChamberAnalyzation/energies.h"

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/inference/external/qpbo.hxx>

const double ENTRANCE_MIN_DISTANCE_TO_END = 20.0; //entrances must have at least this distance from the nearest cave end

void propagateDistance(const ICaveData& data, int vertex, double currentDistance, std::vector<double>& distancesFromEnd)
{
	if (currentDistance < distancesFromEnd.at(vertex) && currentDistance < ENTRANCE_MIN_DISTANCE_TO_END)
	{
		distancesFromEnd.at(vertex) = currentDistance;
		for (int neighbor : data.AdjacentNodes(vertex))
		{
			double distance = (data.VertexPosition(vertex) - data.VertexPosition(neighbor)).norm();
			propagateDistance(data, neighbor, currentDistance + distance, distancesFromEnd);
		}
	}
}

void CurvatureBasedQPBO::FindChambers(const ICaveData& data, std::vector<int>& segmentation, bool verbose)
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
	for (unsigned int i = 0; i < data.NumberOfVertices(); ++i)
		labelSpace.addVariable(2);

	Model gm(labelSpace);

	int nonsubmodular = 0;

	//pairwise factors
	const size_t functionShapePairwise[] = { 2, 2 };
	for (int iEdge = 0; iEdge < data.NumberOfEdges(); ++iEdge)
	{
		size_t v1, v2;
		data.IncidentVertices(iEdge, v1, v2);		

		double derivative = data.CaveSizeDerivative(iEdge);
		double curvature = data.CaveSizeCurvature(iEdge);

		double entranceProbability = ::entranceProbability(curvature * 0.5 * (data.CaveScale(v1) + data.CaveScale(v2)));
		double directionProbability = ::directionProbability(derivative);

		//only allow entrances that have a minimum distance to the cave end
		/*if (distancesFromEnd.at(edge.first) < ENTRANCE_MIN_DISTANCE_TO_END && distancesFromEnd.at(edge.second) < ENTRANCE_MIN_DISTANCE_TO_END)
		entranceProbability = 0;*/

		auto pairwiseFunction = opengm::ExplicitFunction<double>(functionShapePairwise, functionShapePairwise + 2);
		pairwiseFunction(0, 0) = -log(1 - entranceProbability);
		pairwiseFunction(1, 1) = -log(1 - entranceProbability);
		size_t nodes[2];
		if (v1 < v2)
		{
			pairwiseFunction(1, 0) = -log(entranceProbability * directionProbability);
			pairwiseFunction(0, 1) = -log(entranceProbability * (1 - directionProbability));
			nodes[0] = v1;
			nodes[1] = v2;
		}
		else
		{
			pairwiseFunction(0, 1) = -log(entranceProbability * directionProbability);
			pairwiseFunction(1, 0) = -log(entranceProbability * (1 - directionProbability));
			nodes[1] = v1;
			nodes[0] = v2;
		}

		gm.addFactor(gm.addFunction(pairwiseFunction), nodes, nodes + 2);
		if (pairwiseFunction(0, 0) + pairwiseFunction(1, 1) > pairwiseFunction(0, 1) + pairwiseFunction(1, 0))
			++nonsubmodular;
	}


	if(verbose)
		std::cout << "Solving minimization problem (" << gm.numberOfFactors() << " factors, " << nonsubmodular << " non-submodular, " << gm.numberOfVariables() << " variables) ..." << std::endl;


	typedef opengm::external::QPBO<Model> Optimizer;

	Optimizer::Parameter param;
	param.strongPersistency_ = false;
	param.useProbeing_ = false;
	param.useImproveing_ = false;

	Optimizer optimizer(gm, param);
	optimizer.infer();

	if(verbose)
		std::cout << "Generating output..." << std::endl;


	std::vector<size_t> argmin;
	optimizer.arg(argmin);
	segmentation.resize(argmin.size());
	for (int i = 0; i < argmin.size(); ++i)
		segmentation[i] = (argmin[i] == 0 ? 0 : -1);
}