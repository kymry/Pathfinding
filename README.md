This repository contains implementations of various multi-agent and single-agent pathfinding algorithms. 
The folder structure is represented below. All algorithms extend the BFSearch class (Best-first search) which in turn extends the Search class. 

The following algorithms are included:

Single-agent pathfinding:
  AStar
  Focal AStar
  AStart with operator decomposition and independence detection
  SMAStar
  
 Multi-agent pathfinding:
  Conflict based search
  Enhanced conflict based search
  Greedy conflict based search
  Cooperative Astar
  

└── algorithms
    ├── AlgorithmName.java
    ├── Astar
    │   ├── AStarSearch.java
    │   ├── AStarState.java
    │   ├── FocalAStar
    │   │   └── FocalAStarSearch.java
    │   ├── ODStar
    │   │   ├── Move.java
    │   │   ├── ODStarSearch.java
    │   │   └── ODStarState.java
    │   └── SMAStar
    │       ├── SMAStarSearch.java
    │       ├── SMAStarSearchFixedWings.java
    │       ├── SMAStarState.java
    │       ├── SMAStarStateFixedWings.java
    │       └── SMAStarStateParrotDisco.java
    ├── BFSearch.java
    ├── CBS
    │   ├── CBSSearch.java
    │   ├── CBSState.java
    │   ├── ECBS
    │   │   ├── ECBSSearch.java
    │   │   └── ECBSState.java
    │   └── GreedyCBS
    │       ├── GreedyCBSSearch.java
    │       └── GreedyCBSState.java
    ├── CooperativeAStar
    │   └── CooperativeAStarSearch.java
    ├── IndependenceDetecetion
    │   ├── Command.java
    │   ├── IndependenceDetection.java
    │   └── IndependenceDetectionSearch.java
    ├── PathfindingInterface.java
    ├── Search.java
    ├── comparators
    │   ├── AgentConflictComparator.java
    │   ├── AgentPathStartTimeComparator.java
    │   ├── ConflictComparator.java
    │   ├── ConflictCountComparator.java
    │   ├── FValueComparator.java
    │   ├── HValueComparator.java
    │   ├── PairCountComparator.java
    │   └── WeightedFValueComparator.java
    ├── conflicts
    │   ├── AgentConflict.java
    │   ├── AlgorithmConflict.java
    │   ├── ConflictType.java
    │   └── ObstacleConflict.java
    ├── heuristics
    │   ├── DiagonalDistanceHeuristic.java
    │   ├── DiagonalDistanceHeuristicSOC.java
    │   ├── HeuristicInterface.java
    │   ├── NumberOfConflictsHeuristic.java
    │   └── TotalCostHeuristic.java
    └── trees
        ├── Node.java
        └── State.java
