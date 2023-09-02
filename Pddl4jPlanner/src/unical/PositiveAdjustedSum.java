
package unical;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import fr.uga.pddl4j.heuristics.state.RelaxedGraphHeuristic;
import fr.uga.pddl4j.planners.statespace.search.Node;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Condition;
import fr.uga.pddl4j.util.BitVector;

public class PositiveAdjustedSum extends RelaxedGraphHeuristic {

	private static final Logger LOGGER = LogManager.getLogger(PositiveAdjustedSum.class.getName());

	protected PositiveAdjustedSum(Problem problem) {

		super(problem);
		super.setAdmissible(false);
		LOGGER.warn("Using custom heuristics\n");
	}

	@Override
	public int estimate(State state, Condition goal) {

		super.setGoal(goal);
		/*
		 * A relaxed planning graph is a graph that is used to represent the state space of a planning problem.
		 * It is a relaxation of the standard planning graph, in that it does not consider all possible ways
		 * in which the state of the world can change. This makes it more efficient to construct and search,
		 * but it also means that it may not be able to find the optimal solution to the problem.
		 *
		 * The relaxed problem in obtained by removing the delete effects from all actions.
		 *
		 * Get the connectivity level of the relaxed panning graph i.e. every edge connectivity is at least equal to such level
		 */
		final int level = super.expandRelaxedPlanningGraph(state);
		return super.isGoalReachable()
			? this.getSumValue() + (level - this.getMaxValue()) : Integer.MAX_VALUE;
	}

	@Override
	public double estimate(Node node, Condition goal) {

		return estimate((State) node, goal);
	}

	@Override
	protected final int getSumValue() {

		int value = 0;
		// Get goal's positive fluent
		// A fluent is a proposition whose truth value changes during planning process
		final BitVector pGoal = super.getGoal().getPositiveFluents();
		for (int g = pGoal.nextSetBit(0); g >= 0; g = pGoal.nextSetBit(g + 1)) {
			// Get the level of appearance (i.e. the times) the proposition identified by a fluent g appear in the goal
			value += this.pPropLevel[g];
		}
		// Return the sum of the times each proposition appear in the goal
		return value;
	}

	@Override
	protected int getMaxValue() {

		int max = Integer.MIN_VALUE;
		final BitVector pGoal = super.getGoal().getPositiveFluents();
		for (int g = pGoal.nextSetBit(0); g >= 0; g = pGoal.nextSetBit(g + 1)) {
			final int gl = this.pPropLevel[g];
			if (gl > max) {
				max = gl;
			}
		}
		// Return the max times of a proposition appear in the goal
		return max;
	}

}
