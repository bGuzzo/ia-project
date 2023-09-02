
package unical;

import fr.uga.pddl4j.problem.State;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public final class Node extends State {

	private Node parent;

	private int action;

	private double cost;

	private double heuristic;

	private int depth;

	public Node(State state) {

		super(state);
	}

	public Node(State state, Node parent, int action, double cost, double heuristic) {

		super(state);
		this.parent = parent;
		this.action = action;
		this.cost = cost;
		this.heuristic = heuristic;
		this.depth = 0;
	}

	public Node(State state, Node parent, int action, double cost, int depth, double heuristic) {

		super(state);
		this.parent = parent;
		this.action = action;
		this.cost = cost;
		this.depth = depth;
		this.heuristic = heuristic;
	}

}
