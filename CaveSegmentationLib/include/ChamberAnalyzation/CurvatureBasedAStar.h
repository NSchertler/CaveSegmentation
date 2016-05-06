#pragma once

#include "../CaveData.h"
#include "../SignedUnionFind.h"
#include "../GraphProc.h"
#include "energies.h"

#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <queue>

class CurvatureBasedAStar
{
	struct NeighborViaEdge
	{
		int patchIndex;
		int edgeIndex;
		NeighborViaEdge(int patchIndex, int edgeIndex)
			: patchIndex(patchIndex), edgeIndex(edgeIndex) {}
	};
	struct Patch
	{
		double caveSize;
		int representativeVertex;
		std::vector<NeighborViaEdge> neighborPatches;
	};
	struct Edge
	{
		//directed edge; direction refers to an ascending cave size
		int sourcePatch;
		int targetPatch;
		double size;
		double energyEntrance; //the resulting energy if this edge becomes an entrance
		double energyNoEntrance; //the resulting energy if this edge becomes no entrance

		Edge(int sourcePatch, int targetPatch, double size, double energyEntrance, double energyNoEntrance)
			: sourcePatch(sourcePatch), targetPatch(targetPatch), size(size),
			energyEntrance(energyEntrance), energyNoEntrance(energyNoEntrance) {}
	};

	// Represents a state in the A* graph
	class State
	{		
		std::size_t hash;

		// Calculates three bits (plus one parity bit) that represent the two edge states.
		void CalculateHashTriple(bool labeledFirst, bool entranceFirst, bool labeledSecond, bool entranceSecond, size_t& hashTriple, size_t& parity)
		{
			//indexed by [labeled][entrance]
			const int stateNumberTable[2][2] = { { 0, 0 },{ 1, 2 } };
			//indexed by [state1][state2]
			const size_t hashTable[3][3] = { { 0, 1, 2 },{ 3, 4, 5 },{ 6, 7, 0 } };
			const size_t parityTable[3][3] = { { 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 1 } };

			size_t state1 = stateNumberTable[labeledFirst][entranceFirst];
			size_t state2 = stateNumberTable[labeledSecond][entranceSecond];

			hashTriple = hashTable[state1][state2];
			parity = parityTable[state1][state2];
		}

		void UpdateValue(int edge, bool labeled, bool entrance)
		{
			//get old hash triple
			const int edgesPerRow = sizeof(size_t) * 8 / 3;
			int hashTripleLocation = (edge % edgesPerRow) / 2;

			size_t oldHash = (hash >> (hashTripleLocation * 3)) & (7);
			size_t oldParity = hash >> (sizeof(size_t) * 8 - 1);

			//update values
			edgeLabeled.at(edge) = true;
			edgeEntrance.at(edge) = entrance;

			//update hash
			size_t newHash, newParity;
			int tripleComponent1 = 2 * hashTripleLocation;
			int tripleComponent2 = 2 * hashTripleLocation + 1;
			CalculateHashTriple(edgeLabeled.at(tripleComponent1), edgeEntrance.at(tripleComponent1),
				edgeLabeled.at(tripleComponent2), edgeEntrance.at(tripleComponent2), newHash, newParity);

			hash ^= (oldHash ^ newHash) << (hashTripleLocation * 3);
			hash ^= (oldParity ^ newParity) << (sizeof(size_t) * 8 - 1);
		}

		template <bool reverseDirection>
		void PushEntrance(int entrance, const std::vector<Patch>& patches, const std::vector<Edge>& edges)
		{
			auto& edge = edges.at(entrance);

			//entrance switches from "entrance" to "no entrance"
			currentEnergy.Remove(entrance, edge.energyEntrance);
			currentEnergy.Add(entrance, edge.energyNoEntrance);
			minEnergyAtTarget.Remove(entrance, std::min(edge.energyEntrance, edge.energyNoEntrance));
			minEnergyAtTarget.Add(entrance, edge.energyNoEntrance);			

			entrances.erase(entrance);
			UpdateValue(entrance, true, false);

			//the vertex across which the entrance is pushed
			int vertexNewInChamber = (reverseDirection ? edge.targetPatch : edge.sourcePatch);
			//the vertex that was in the chamber before pushing the entrance
			int vertexAlreadyInChamber = (reverseDirection ? edge.sourcePatch : edge.targetPatch);			

			std::vector<int> edgesThatNeedPushing;			
			for (auto& neighbor : patches.at(vertexNewInChamber).neighborPatches)
			{
				if (neighbor.patchIndex == vertexAlreadyInChamber)
					continue; //wrong propagation direction

				auto& neighborEdge = edges.at(neighbor.edgeIndex);

				if (edgeLabeled.at(neighbor.edgeIndex))
				{
					//neighbor edge is already labeled
					//must be another entrance

					//merge the entrances					
					UpdateValue(neighbor.edgeIndex, true, false);
					currentEnergy.Remove(neighbor.edgeIndex, neighborEdge.energyEntrance);
					currentEnergy.Add(neighbor.edgeIndex, neighborEdge.energyNoEntrance);
					minEnergyAtTarget.Remove(entrance, std::min(neighborEdge.energyEntrance, neighborEdge.energyNoEntrance));
					minEnergyAtTarget.Add(neighbor.edgeIndex, neighborEdge.energyNoEntrance);

					entrances.erase(neighbor.edgeIndex);
				}
				else
				{
					//neighbor edge is still unlabeled
					UpdateValue(neighbor.edgeIndex, true, true);
					currentEnergy.Add(neighbor.edgeIndex, neighborEdge.energyEntrance);
					entrances.insert(neighbor.edgeIndex);

					//check if the direction is ok
					if (neighborEdge.targetPatch != vertexNewInChamber)
					{
						//direction is not ok, edge needs to be pushed further
						edgesThatNeedPushing.push_back(neighbor.edgeIndex);
					}					
				}
			}
			//At this point, entrances are closed borders around chamber, but some may be in the wrong direction
			for (int edgeIndex : edgesThatNeedPushing)
				PushEntrance<true>(edgeIndex, patches, edges);
		}

		//Accumulates values and keeps track of infinite parts
		template <typename T>
		class Accumulator
		{
		private:
			T nonInfinitySum;
			T finalSum;
			std::unordered_set<int> infiniteParts;			

		public:			

			Accumulator() : nonInfinitySum(0), finalSum(0) {}

			void Add(int index, T value)
			{
				if (isinf(value))
					infiniteParts.insert(index);
				else
					nonInfinitySum += value;
				finalSum += value;
			}

			void Remove(int index, T value)
			{
				if (isinf(value))
				{
					infiniteParts.erase(index);
					if (infiniteParts.size() == 0)
						finalSum = nonInfinitySum;
				}
				else
				{
					nonInfinitySum -= value;
					finalSum -= value;
				}
			}

			T Sum() const { return finalSum; }

			bool operator==(const Accumulator<T>& other) const { return finalSum == other.finalSum; }
			bool operator!=(const Accumulator<T>& other) const { return finalSum != other.finalSum; }
			bool operator<(const Accumulator<T>& other) const { return finalSum < other.finalSum; }
			bool operator>(const Accumulator<T>& other) const { return finalSum > other.finalSum; }
			bool operator>=(const Accumulator<T>& other) const { return finalSum >= other.finalSum; }
			std::ostream& operator<<(std::ostream& s) const { return s << finalSum; }
		};

	public:
		std::vector<bool> edgeLabeled;
		std::vector<bool> edgeEntrance;
		std::unordered_set<int> seeds; //unused seed candidates
		std::unordered_set<int> entrances; //indices of edges that are labeled as entrances

		Accumulator<double> currentEnergy; //the energy that has been paid so far by labeled edges
		Accumulator<double> minEnergyAtTarget; //lower bound for the resulting energy at the target state

		//Initialize the state with all edges unlabeled
		State(const std::vector<Edge>& edges, const std::unordered_set<int>& seeds)
			: edgeLabeled(edges.size(), false), edgeEntrance(edges.size(), false), seeds(seeds),
			hash(0)
		{
			for (int i = 0; i < edges.size(); ++i)
			{
				auto& e = edges.at(i);
				minEnergyAtTarget.Add(i, std::min(e.energyEntrance, e.energyNoEntrance));
			}
		}

		State(const State& copy)
			: edgeLabeled(copy.edgeLabeled),
			edgeEntrance(copy.edgeEntrance),
			seeds(copy.seeds),
			entrances(copy.entrances),
			hash(copy.hash), currentEnergy(copy.currentEnergy), minEnergyAtTarget(copy.minEnergyAtTarget)
		{ }

		State& operator=(State&& movedFrom)
		{
			edgeLabeled = std::move(movedFrom.edgeLabeled);
			edgeEntrance = std::move(movedFrom.edgeEntrance);
			seeds = std::move(movedFrom.seeds);
			entrances = std::move(movedFrom.entrances);
			hash = movedFrom.hash;
			currentEnergy = movedFrom.currentEnergy;
			minEnergyAtTarget = movedFrom.minEnergyAtTarget;

			return *this;
		}

		bool operator<(const State& other) const
		{
			if (minEnergyAtTarget != other.minEnergyAtTarget)
				return minEnergyAtTarget < other.minEnergyAtTarget;
			else
				return currentEnergy < other.currentEnergy;
		}

		bool operator>(const State& other) const
		{
			if (minEnergyAtTarget != other.minEnergyAtTarget)
				return minEnergyAtTarget > other.minEnergyAtTarget;
			else
				return currentEnergy > other.currentEnergy;
		}

		bool operator==(const State& rhs) const
		{
			if (hash != rhs.hash || currentEnergy != rhs.currentEnergy)
				return false;
			if (edgeLabeled.size() != rhs.edgeLabeled.size())
				return false;
			for (int i = 0; i < edgeLabeled.size(); ++i)
			{
				if (edgeLabeled.at(i) != rhs.edgeLabeled.at(i) || edgeEntrance.at(i) != rhs.edgeEntrance.at(i))
					return false;
			}
			return true;
		}
		
		State* SetUnlabeledEdgesToNoEntrances(const std::vector<Edge>& edges) const
		{
			State* copy = new State(*this);
			for (int i = 0; i < edges.size(); ++i)
			{
				if (!edgeLabeled.at(i))
				{
					copy->UpdateValue(i, true, false);
					copy->currentEnergy.Add(i, edges.at(i).energyNoEntrance);
				}
			}
			copy->minEnergyAtTarget = copy->currentEnergy;
			return copy;
		}

		bool CanPlaceSeed(int seed, const std::vector<Patch>& patches) const
		{
			auto& neighbors = patches.at(seed).neighborPatches;
			if (neighbors.size() == 0 || !edgeLabeled.at(neighbors.front().edgeIndex))
				return true;
			return false;
		}

		State* PlaceSeed(int seed, const std::vector<Patch>& patches, const std::vector<Edge>& edges) const
		{
			State* copy = new State(*this);
			for (auto& neighbor : patches.at(seed).neighborPatches)
			{
				//All edges of a seed are incoming. Set them all to entrances.
				int eIndex = neighbor.edgeIndex;
				auto& edge = edges.at(eIndex);
				copy->UpdateValue(eIndex, true, true);
				copy->currentEnergy.Add(eIndex, edge.energyEntrance);
				copy->entrances.insert(eIndex);
			}
			copy->seeds.erase(seed);
			if (abs(copy->CalculateHeuristic(edges) - copy->minEnergyAtTarget.Sum()) > 0.01)
			{
				std::cout << "Wrong heuristic.";
				system("PAUSE");
			}
			if (abs(copy->CalculateEnergy(edges) - copy->currentEnergy.Sum()) > 0.01)
			{
				std::cout << "Wrong energy.";
				system("PAUSE");
			}
			return copy;
		}

		State* AdvanceEntrance(int entrance, const std::vector<Patch>& patches, const std::vector<Edge>& edges) const
		{
			State* copy = new State(*this);
			copy->PushEntrance<false>(entrance, patches, edges);
			if (abs(copy->CalculateHeuristic(edges) - copy->minEnergyAtTarget.Sum()) > 0.01)
			{
				std::cout << "Wrong heuristic.";
				system("PAUSE");
			}
			if (abs(copy->CalculateEnergy(edges) - copy->currentEnergy.Sum()) > 0.01)
			{
				std::cout << "Wrong energy.";
				system("PAUSE");
			}
			return copy;
		}

		double CalculateEnergy(const std::vector<Edge>& edges) const
		{
			double energy = 0;
			for (int i = 0; i < edges.size(); ++i)
			{
				if (edgeLabeled.at(i))
					energy += (edgeEntrance.at(i) ? edges.at(i).energyEntrance : edges.at(i).energyNoEntrance);
			}
			return energy;
		}

		double CalculateHeuristic(const std::vector<Edge>& edges) const
		{
			double energy = 0;
			for (int i = 0; i < edges.size(); ++i)
			{
				if (edgeLabeled.at(i) && !edgeEntrance.at(i))
					energy += (edgeEntrance.at(i) ? edges.at(i).energyEntrance : edges.at(i).energyNoEntrance);
				else
					energy += std::min(edges.at(i).energyEntrance, edges.at(i).energyNoEntrance);
			}
			return energy;
		}

		struct Hash
		{
			size_t operator() (const State* state) const { return state->hash; }
		};

		struct Equals
		{
			bool operator() (const State* lhs, const State* rhs) const { return *lhs == *rhs; }
		};

		struct EnergyAtTargetBasedPriority
		{
			bool operator() (const State* lhs, const State* rhs) const { return *lhs > *rhs; }
		};
	};

	template <bool debug = false>
	static void VisualizeState(const State* state, const std::vector<Patch>& patches, const std::vector<Edge>& edges, const std::vector<CurveSkeleton::Vertex>& vertices, const std::string& name)
	{
		if(debug)
			return;
		{
			std::ofstream dot("layout.dot");
			int oldPrec = dot.precision();
			dot << "digraph G {	graph[splines = \"false\" inputscale=10];";
			dot << "node[shape=\"point\"];";
			for (int i = 0; i < patches.size(); ++i)
			{
				auto& v = vertices.at(patches.at(i).representativeVertex);
				dot << i << "[pos=\"" << v.position.x() << "," << v.position.y() << "!\"];";
			}
			for (int i = 0; i < edges.size(); ++i)
			{
				auto& e = edges.at(i);
				dot << e.sourcePatch << "->" << e.targetPatch << "[color=";
				if (state->edgeLabeled.at(i))
					if (state->edgeEntrance.at(i))
						dot << "red";
					else
						dot << "black";
				else
					dot << "gray";				
				dot.precision(2);
				dot << ",label=\"" << (edges.at(i).energyEntrance - edges.at(i).energyNoEntrance) << "\"];";
				dot.precision(oldPrec);
			}
			dot << "}";
		}
		std::string call = "\"\"C:\\Program Files (x86)\\Graphviz2.38\\bin\\neato\" -Tpng layout.dot -o \"" + name + "\"\"";
		system(call.c_str());
	}

public:
	static void FindChambers(const CaveData& data, std::vector<int>& segmentation)
	{
		//Contract the graph at non-local maximum edges
		std::cout << "Contracting graph..." << std::endl;

		std::vector<bool> keepEdge(data.skeleton->edges.size());

		//Find local maxima
		for (int i = 0; i < data.skeleton->edges.size(); ++i)
		{
			auto& edge = data.skeleton->edges.at(i);

			double edgeValue = data.caveSizeCurvaturesPerEdge.at(i);

			//check incoming and outgoing neighbors
			bool isMaximum = checkEdgeMaximum(data.adjacency, data.caveSizeCurvaturesPerEdge, data.vertexPairToEdge, edge.first, edge.second, edgeValue);
			if (isMaximum)
				isMaximum = checkEdgeMaximum(data.adjacency, data.caveSizeCurvaturesPerEdge, data.vertexPairToEdge, edge.second, edge.first, edgeValue);

			keepEdge.at(i) = isMaximum;
		}

		//TODO: check if we need to keep at least one edge per branch (from crossing to crossing) to preserve topology

		//contract vertices and keep track of maximum cave size
		SignedUnionFind<false> vertexPatches;
		vertexPatches.addItems(data.skeleton->vertices.size(), false);
		std::vector<double> ufCaveSizeMaxima(data.caveSizes.begin(), data.caveSizes.end());

		std::vector<int> preservedEdges;
		for (int i = 0; i < data.skeleton->edges.size(); ++i)
		{
			auto& edge = data.skeleton->edges.at(i);
			if (keepEdge.at(i))
			{
				preservedEdges.push_back(i);
			}
			else
			{
				auto representative1 = vertexPatches.getRepresentative(edge.first);
				auto representative2 = vertexPatches.getRepresentative(edge.second);
				double newMaxCaveSize = std::max(ufCaveSizeMaxima.at(representative1), ufCaveSizeMaxima.at(representative2));
				auto newRepresentative = vertexPatches.merge(representative1, representative2);
				ufCaveSizeMaxima.at(newRepresentative) = newMaxCaveSize;
			}
		}
		
		//Number the patches
		int patchCount = 0;
		std::map<int, int> vertexToPatch;
		
		std::vector<Patch> patches;
		std::vector<Edge> edges;
		std::unordered_set<int> seedCandidates; //indices of vertices with solely incoming edges
		for (unsigned int representative : vertexPatches.roots())
		{
			vertexToPatch[representative] = patchCount++;
			patches.emplace_back();
			Patch& patch = patches.back();
			patch.caveSize = ufCaveSizeMaxima.at(representative);
			patch.representativeVertex = representative;
			seedCandidates.insert(patches.size() - 1);
		}
		for (int i : preservedEdges)
		{
			auto& edge = data.skeleton->edges.at(i);
			int sourcePatch = vertexToPatch.at(vertexPatches.getRepresentative(edge.first));
			int targetPatch = vertexToPatch.at(vertexPatches.getRepresentative(edge.second));

			//evaluate direction based on first derivative of cave size
			double derivative = data.caveSizeDerivativesPerEdge.at(i);
			if (derivative < 0)
			{
				std::swap(sourcePatch, targetPatch);
				derivative *= -1;
			}

			if (sourcePatch == targetPatch)
				std::cout << "Loop edge!!." << std::endl;

			double entranceProb = entranceProbability(data.caveSizeCurvaturesPerEdge.at(i));
			double size = (data.caveSizes.at(edge.first) + data.caveSizes.at(edge.second)) / 2;

			edges.emplace_back(sourcePatch, targetPatch, size, -log(entranceProb), -log(1 - entranceProb));
			patches.at(sourcePatch).neighborPatches.emplace_back(targetPatch, edges.size() - 1);
			patches.at(targetPatch).neighborPatches.emplace_back(sourcePatch, edges.size() - 1);

			//Remove the patch with outgoing edges from the list of seed candidates
			auto seedIt = seedCandidates.find(sourcePatch);
			if (seedIt != seedCandidates.end())
				seedCandidates.erase(seedIt);
		}		

		double minimumEnergy = std::numeric_limits<double>::infinity();
		State* optimalState = nullptr;

		{
			std::unordered_set<const State*, State::Hash, State::Equals> visitedStates;
			std::priority_queue<const State*, std::vector<const State*>, State::EnergyAtTargetBasedPriority> openStates;

			while (seedCandidates.size() > 1)
				seedCandidates.erase(*seedCandidates.begin());
			auto initialState = new State(edges, seedCandidates); //Start with an initial state
			visitedStates.insert(initialState);
			openStates.push(initialState);

			int iteration = 0;
			while (!openStates.empty())
			{
				const State* currentState = openStates.top();
				openStates.pop();

				VisualizeState(currentState, patches, edges, data.skeleton->vertices, "Iteration" + std::to_string(iteration) + "_Base.png");

				std::cout << "current optimum: " << minimumEnergy << "; current energy: " << currentState->currentEnergy.Sum() << "; lower bound: " << currentState->minEnergyAtTarget.Sum() << "; open states: " << openStates.size() << "; seeds: " << currentState->seeds.size() << "; entrances: " << currentState->entrances.size() << "     ";
				std::cout << std::endl;

				if (currentState->minEnergyAtTarget.Sum() >= 0.99 * minimumEnergy)
					break; //we cannot find a better state than the currently optimal one				

				int step = 0;

				//Place seeds
				for (int seed : currentState->seeds)
				{
					if (!currentState->CanPlaceSeed(seed, patches))
						continue;
					State* newState = currentState->PlaceSeed(seed, patches, edges);
					if (newState->minEnergyAtTarget.Sum() < minimumEnergy && visitedStates.find(newState) == visitedStates.end())
					{
						VisualizeState(newState, patches, edges, data.skeleton->vertices, "Iteration" + std::to_string(iteration) + "_Step" + std::to_string(step++) + "_MinHeur" + std::to_string(newState->minEnergyAtTarget.Sum()) + ".png");
						//we haven't visited this state before
						visitedStates.insert(newState);
						openStates.push(newState);
					}
					else
						delete newState;
				}

				//Push entrances
				for (int entrance : currentState->entrances)
				{
					State* newState = currentState->AdvanceEntrance(entrance, patches, edges);
					if (newState->minEnergyAtTarget.Sum() < minimumEnergy && visitedStates.find(newState) == visitedStates.end())
					{
						VisualizeState(newState, patches, edges, data.skeleton->vertices, "Iteration" + std::to_string(iteration) + "_Step" + std::to_string(step++) + "_MinHeur" + (isinf(newState->minEnergyAtTarget.Sum()) ? "INF" : std::to_string(newState->minEnergyAtTarget.Sum())) + ".png");
						//we haven't visited this state before
						visitedStates.insert(newState);
						openStates.push(newState);
					}
					else
						delete newState;
				}

				//Directly go to the target
				State* target = currentState->SetUnlabeledEdgesToNoEntrances(edges);
				if (target->currentEnergy.Sum() < minimumEnergy)
				{
					optimalState = target;
					minimumEnergy = optimalState->currentEnergy.Sum();
				}
				else
					delete target;

				iteration++;
			}

			std::cout << "Total states: " << visitedStates.size() << std::endl;

			//Clean up
			for (auto state : visitedStates)
				delete state;
		}
		std::cout << std::endl;
		if (optimalState)
		{
			std::cout << "Optimal state: " << optimalState->currentEnergy.Sum() << std::endl;
			VisualizeState<false>(optimalState, patches, edges, data.skeleton->vertices, "optimum.png");
		}
		else
			std::cout << "No optimal solution exists." << std::endl;
		
	}
};