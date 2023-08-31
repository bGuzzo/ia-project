
package pddl_ia;

import fr.uga.pddl4j.heuristics.state.RelaxedGraphHeuristic;
import fr.uga.pddl4j.planners.statespace.search.Node;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Condition;

public class CustomHeuristics extends RelaxedGraphHeuristic {

	private static final long serialVersionUID = -5088152190199562298L;

	protected CustomHeuristics(Problem arg0) {

		super(arg0);
		// TODO Auto-generated constructor stub
	}

	@Override
	public int estimate(State arg0, Condition arg1) {

		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double estimate(Node arg0, Condition arg1) {

		// TODO Auto-generated method stub
		return 0;
	}

}
