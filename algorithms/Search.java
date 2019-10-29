package algorithms;

public abstract class Search {
	
	protected long _timeLimit;
	protected long _initialTime;
	protected boolean _noSolution = false;
	
	
	public abstract boolean findPath(long timeLimit, long initialTime);
	
	public boolean isNoSolution() {
		return _noSolution;
	}

}
