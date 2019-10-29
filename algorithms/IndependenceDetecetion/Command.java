package algorithms.IndependenceDetecetion;

/**
 *
 * Implemented by any class that utilizes the Independence Detection framework
 * execute: main method to drive the path finding algorithm
 * updateConflicts: method to update conflicts.
 * findConflicts: method to find new conflicts
 *
 * Implements the command design pattern
 */
public interface Command {

    /**
     * @param updatePaths If true, agent paths will be altered regardless of final sum of costs(SOC)
     * If false, agent paths will only be altered if SOC <= optimalCost * weight
     * @param group1 new paths for these agents ONLY will be found
     * @param group2 paths for these agents will be set as obstacles in the MAPF algorithm. new paths will NOT be found
     * @return true if SOC <= optimal costs * weight
     */
    boolean execute(Boolean updatePaths, Integer group1, Integer group2);

    /**
     * Removes all conflicts containing agent(s) in the provided group
     * Sets Agent path variable freeOfConflicts to true
     * @param group group of agents whose conflicts to remove
     */
    void updateConflicts(Integer group);

    /**
     * finds all new conflicts with the specified group of agents
     * @param group group of agents whose conflicts will be found
     * @param findAllConflicts if true, all conflicts between two agents will be found. else, only the first conflict will
     *                         be found and returned
     */
    void findConflicts(Integer group, Boolean findAllConflicts);
}
