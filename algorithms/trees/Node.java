package algorithms.trees;

public class Node implements Comparable<Node> {
	
	private Node _root;
	private Node _parent;
	private State _state;
	private boolean _timeout;
	
	
	public Node(Node parent, State state) {
		_parent = parent;
		_state = state;
		_timeout = false;
		if(parent == null) {
			_root = this; 
		}
		else {
			_root = parent.getRoot();
		}
	}
	
	@Override
	public int compareTo(Node o) {
		return _state.compareTo(o.getState());
	}

	@Override
	public boolean equals(Object o) {
		Node other = (Node)o;
		return _state.equals(other.getState());
	}
	
	@Override
	public int hashCode() {
		return _state.hashCode();
	}
	
	@Override
	public String toString() {
		if(_parent == null) {
			return "State: " + _state.toString();
		}
		return "State: " + _state.toString();// + "\nParent: "+ _parent.toString();
	}
	
	public Node clone() {
		return new Node(_parent, _state.clone());
	}
	
	public Node getRoot() {
		return _root;
	}
	
	public Node getParent() {
		return _parent;
	}
	
	public State getState() {
		return _state;
	}
	
	public void setState(State state) {
		_state = state;
	}

	public boolean isTimeout() {
		return _timeout;
	}

	public void setTimeout(boolean timeout) {
		this._timeout = timeout;
	}

	public void setParent(Node parent) {
		this._parent = parent;
	}

}
