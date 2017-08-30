#include "ChamberAnalyzation/SizeBasedQPBO.h"

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/inference/external/qpbo.hxx>

#include <iostream>

void SizeBasedQPBO::FindChambers(const ICaveData& data, std::vector<int>& segmentation, bool verbose)
{
	if(verbose)
		std::cout << "Building graphical model..." << std::endl;

	opengm::DiscreteSpace<> labelSpace;
	typedef opengm::GraphicalModel<double, opengm::Adder> Model;

	//Label 0: Cave
	//Label 1: Passage
	for (unsigned int i = 0; i < data.NumberOfVertices(); ++i)
		labelSpace.addVariable(2);

	Model gm(labelSpace);

	const double SIZE_THRESHOLD_PASSAGE = 8.0;
	const double SIZE_THRESHOLD_CAVE = 15.0;
	const double SOLID_ANGLE_THRESHOLD_PASSAGE = 2.0;
	const double SOLID_ANGLE_THRESHOLD_CAVE = 4.0;
	const double MIN_CAVE_PROBABILITY = 0.1;
	const double MAX_CAVE_PASSAGE_ENERGY = log((1 - MIN_CAVE_PROBABILITY) / MIN_CAVE_PROBABILITY); //the difference of cave energy and passage energy for very small sizes, per length unit

																								   //Unary factors
	const size_t functionShapeUnary[] = { 2 };
	for (int iVert = 0; iVert < data.NumberOfVertices(); ++iVert)
	{
		auto unaryFunction = opengm::ExplicitFunction<double>(functionShapeUnary, functionShapeUnary + 1);

		double caveProbability =
			std::max(MIN_CAVE_PROBABILITY, std::min(1.0 - MIN_CAVE_PROBABILITY,
				MIN_CAVE_PROBABILITY + (1 - 2 * MIN_CAVE_PROBABILITY) * (data.CaveSize(iVert) - SOLID_ANGLE_THRESHOLD_PASSAGE) / (SOLID_ANGLE_THRESHOLD_CAVE - SOLID_ANGLE_THRESHOLD_PASSAGE)));

		unaryFunction(0) = -log(caveProbability) * 2 * data.NodeRadius(iVert);
		unaryFunction(1) = -log(1 - caveProbability) * 2 * data.NodeRadius(iVert);
		size_t node = iVert++;
		gm.addFactor(gm.addFunction(unaryFunction), &node, &node + 1);
	}

	//Pairwise factors
	const double DERIVATIVE_THRESHOLD = 0.5;
	const double SEGMENT_MIN_LENGTH = 10.0;

	const size_t functionShapePairwise[] = { 2, 2 };
	
	for (int iEdge = 0; iEdge < data.NumberOfEdges(); ++iEdge)
	{
		//auto v1 = skeleton->vertices[edge.first];
		//auto v2 = skeleton->vertices[edge.second];
		size_t i1, i2;
		data.IncidentVertices(iEdge, i1, i2);
		if (i2 < i1) { i1 = i2; i2 = i1; }
		//double edgeLength = (v1.position - v2.position).norm();
		//double derivative = (maxDistances[i1] - maxDistances[i2]) / edgeLength;

		auto pairwiseFunction = opengm::ExplicitFunction<double>(functionShapePairwise, functionShapePairwise + 2);
		/*if (derivative > DERIVATIVE_THRESHOLD)
		{
		//most likely a transition from passage to cave
		pairwiseFunction(0, 0) = (derivative - DERIVATIVE_THRESHOLD) * edgeLength;
		pairwiseFunction(1, 1) = (derivative - DERIVATIVE_THRESHOLD) * edgeLength;
		pairwiseFunction(0, 1) = std::numeric_limits<double>::infinity();
		pairwiseFunction(1, 0) = 0;
		}
		else if (derivative < -DERIVATIVE_THRESHOLD)
		{
		//most likely a transition from cave to passage
		pairwiseFunction(0, 0) = (-derivative - DERIVATIVE_THRESHOLD) * edgeLength;
		pairwiseFunction(1, 1) = (-derivative - DERIVATIVE_THRESHOLD) * edgeLength;
		pairwiseFunction(0, 1) = 0;
		pairwiseFunction(1, 0) = std::numeric_limits<double>::infinity();
		}
		else
		{
		//most likely no transition at all
		pairwiseFunction(0, 0) = 0;
		pairwiseFunction(1, 1) = 0;
		pairwiseFunction(0, 1) = edgeLength * std::abs(DERIVATIVE_THRESHOLD - derivative);
		pairwiseFunction(1, 0) = edgeLength * std::abs(DERIVATIVE_THRESHOLD - derivative);
		}*/

		//Smooth term
		pairwiseFunction(0, 0) = 0;
		pairwiseFunction(1, 1) = 0;
		pairwiseFunction(0, 1) = MAX_CAVE_PASSAGE_ENERGY * SEGMENT_MIN_LENGTH;
		pairwiseFunction(1, 0) = MAX_CAVE_PASSAGE_ENERGY * SEGMENT_MIN_LENGTH;

		size_t nodes[] = { i1, i2 };
		gm.addFactor(gm.addFunction(pairwiseFunction), nodes, nodes + 2);
	}

	if(verbose)
		std::cout << "Solving minimization problem..." << std::endl;

	typedef opengm::external::QPBO<Model> Optimizer;

	Optimizer::Parameter param;
	param.strongPersistency_ = false;
	param.useProbeing_ = true;
	param.useImproveing_ = true;

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