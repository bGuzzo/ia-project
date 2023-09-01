
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
		final BitVector pGoal = super.getGoal().getPositiveFluents();
		LOGGER.info("pGoal vector = " + pGoal + "\n");
		for (int g = pGoal.nextSetBit(0); g >= 0; g = pGoal.nextSetBit(g + 1)) {
			LOGGER.info("pGoal g = " + g + "\n");
			LOGGER.info("pPropLevel[g] = " + this.pPropLevel[g] + "\n");
			value += this.pPropLevel[g];
		}
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
		return max;
	}

}
