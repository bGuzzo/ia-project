
package unical;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import lombok.Getter;
import lombok.Setter;
import picocli.CommandLine;

@Getter
@Setter
@CommandLine.Command(name = "AstarCustom")
public class AstarCustom extends AbstractPlanner {

	private static final Logger LOGGER = LogManager.getLogger(PositiveAdjustedSum.class.getName());

	// Default configuration
	public static final String HEURISTIC_SETTING = "HEURISTIC";
	public static final StateHeuristic.Name DEFAULT_HEURISTIC = StateHeuristic.Name.AJUSTED_SUM;
	public static final String WEIGHT_HEURISTIC_SETTING = "WEIGHT_HEURISTIC";
	public static final double DEFAULT_WEIGHT_HEURISTIC = 1.0;
	public static final int DEFAULT_MAX_DEPTH = 100;

	// Planner configuration
	private double heuristicWeight;
	private StateHeuristic.Name heuristic;
	private int maxDepth;
	private boolean customHeuristics;
	private long beginMemory;
	private long endMemory;

	public AstarCustom() {

		this(AstarCustom.getDefaultConfiguration());
		this.maxDepth = DEFAULT_MAX_DEPTH;
		this.customHeuristics = false;

	}

	public AstarCustom(final PlannerConfiguration configuration) {

		super();
		this.setConfiguration(configuration);
	}

	@Override
	public void setConfiguration(final PlannerConfiguration configuration) {

		super.setConfiguration(configuration);
		if (configuration.getProperty(AstarCustom.WEIGHT_HEURISTIC_SETTING) == null) {
			this.setHeuristicWeight(AstarCustom.DEFAULT_WEIGHT_HEURISTIC);
		}
		else {
			this.setHeuristicWeight(
				Double.parseDouble(
					configuration.getProperty(AstarCustom.WEIGHT_HEURISTIC_SETTING)));
		}
		if (configuration.getProperty(AstarCustom.HEURISTIC_SETTING) == null) {
			this.setHeuristic(AstarCustom.DEFAULT_HEURISTIC);
		}
		else {
			this.setHeuristic(
				StateHeuristic.Name.valueOf(
					configuration.getProperty(AstarCustom.HEURISTIC_SETTING)));
		}
	}

	public void setHeuristicWeight(final double weight) {

		if (weight <= 0) {
			throw new IllegalArgumentException("Weight <= 0");
		}
		this.heuristicWeight = weight;
	}

	@Override
	public Problem instantiate(DefaultParsedProblem problem) {

		final Problem pb = new DefaultProblem(problem);
		pb.instantiate();
		return pb;
	}

	@Override
	public boolean hasValidConfiguration() {

		return super.hasValidConfiguration() && this.getHeuristicWeight() > 0.0
				&& this.getHeuristic() != null;
	}

	public static PlannerConfiguration getDefaultConfiguration() {

		PlannerConfiguration config = Planner.getDefaultConfiguration();
		config.setProperty(AstarCustom.HEURISTIC_SETTING, AstarCustom.DEFAULT_HEURISTIC.toString());
		config.setProperty(
			AstarCustom.WEIGHT_HEURISTIC_SETTING,
			Double.toString(AstarCustom.DEFAULT_WEIGHT_HEURISTIC));
		return config;
	}

	@Override
	public PlannerConfiguration getConfiguration() {

		final PlannerConfiguration config = super.getConfiguration();
		config.setProperty(AstarCustom.HEURISTIC_SETTING, this.getHeuristic().toString());
		config.setProperty(
			AstarCustom.WEIGHT_HEURISTIC_SETTING, Double.toString(this.getHeuristicWeight()));
		return config;
	}

	private StateHeuristic getHeuristics(Problem problem) {

		if (customHeuristics) {
			// Instantiate custom heuristics
			return new PositiveAdjustedSum(problem);
		}
		else {
			// Instantiate default heuristics
			return StateHeuristic.getInstance(this.getHeuristic(), problem);
		}
	}

	@Override
	public Plan solve(final Problem problem) {

		LOGGER.info("* Starting A* search \n");
		final long begin = System.currentTimeMillis();
		// Start search with custom A* algorithm
		final Plan plan = this.astar(problem);
		final long end = System.currentTimeMillis();
		if (plan != null) {
			// Search successes set statistics
			LOGGER.info("* A* search succeeded \n");
			this.getStatistics().setTimeToSearch(end - begin);
			this.getStatistics().setMemoryUsedToSearch(Math.abs(endMemory - beginMemory));
		}
		else {
			LOGGER.info("* A* search failed \n");
		}
		// If fail return null plan
		return plan;
	}

	public Plan astar(Problem problem) {

		// Check if well formed
		if (!this.isSupported(problem)) {
			return null;
		}
		Runtime runtime = Runtime.getRuntime();
		this.beginMemory = runtime.totalMemory() - runtime.freeMemory();
		final StateHeuristic heuristic = this.getHeuristics(problem);
		// Build the initial state of the problem
		final State init = new State(problem.getInitialState());
		// Define a set of already esplored node
		final Set<Node> exploredNode = new HashSet<>();
		final double weight = this.getHeuristicWeight();
		// Define a fringe as priority queue based of weighted-f function
		final PriorityQueue<Node> fringe = new PriorityQueue<>(100, new Comparator<Node>() {

			@Override
			public int compare(Node n1, Node n2) {

				double f1 = weight * n1.getHeuristic() + n1.getCost();
				double f2 = weight * n2.getHeuristic() + n2.getCost();
				return Double.compare(f1, f2);
			}
		});
		// Build the root of the searching three
		final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));
		fringe.add(root);
		Plan plan = null;
		// Start the search
		while (!fringe.isEmpty() && plan == null) {
			// Remove first node from the fringe
			final Node current = fringe.poll();
			// Ignore the node if depth exceeded
			if (current.getDepth() > this.maxDepth) {
				continue;
			}
			exploredNode.add(current);
			if (current.satisfy(problem.getGoal())) {
				// Problem solved
				LOGGER.info("Found goal at depth " + current.getDepth() + "\n");
				this.endMemory = runtime.totalMemory() - runtime.freeMemory();
				return this.extractPlan(current, problem);
			}
			else {
				for (int i = 0; i < problem.getActions().size(); i++) {
					// We get the actions of the problem
					Action a = problem.getActions().get(i);
					if (a.isApplicable(current)) {
						// If action a is applicable to current node generate child node
						Node next = new Node(current);
						next.setDepth(current.getDepth() + 1);
						// Get requirements and effects of the action a applied
						final List<ConditionalEffect> effects = a.getConditionalEffects();
						for (ConditionalEffect ce : effects) {
							// In the current state satisfy all the preconditions apply the effects on the child node
							if (current.satisfy(ce.getCondition())) {
								next.apply(ce.getEffect());
							}
						}
						// if the new node (sate) is not explored yet add the child into the fringe
						if (!exploredNode.contains(next)) {
							// Define cost ad the number of the nodes already explored
							next.setCost(current.getCost() + 1);
							next.setParent(current);
							// Set the applied action (encoded as integer) that generate child node
							next.setAction(i);
							// Set the estimate distance to the goal according to the current heuristics
							next.setHeuristic(heuristic.estimate(next, problem.getGoal()));
							// Add child node to the fringe to be explored soon
							fringe.add(next);
						}
					}
				}
			}
		}
		this.endMemory = runtime.totalMemory() - runtime.freeMemory();
		return plan;
	}

	private Plan extractPlan(final Node node, final Problem problem) {

		Node n = node;
		final Plan plan = new SequentialPlan();
		while (n.getAction() != -1) {
			final Action a = problem.getActions().get(n.getAction());
			plan.add(0, a);
			n = n.getParent();
		}
		return plan;
	}

	@Override
	public boolean isSupported(Problem problem) {

		return (problem.getRequirements().contains(RequireKey.ACTION_COSTS)
				|| problem.getRequirements().contains(RequireKey.CONSTRAINTS)
				|| problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
				|| problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
				|| problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
				|| problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
				|| problem.getRequirements().contains(RequireKey.FLUENTS)
				|| problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
				|| problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
				|| problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
				|| problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
				|| problem.getRequirements().contains(RequireKey.PREFERENCES)
				|| problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
				|| problem.getRequirements().contains(RequireKey.HIERARCHY)) ? false : true;
	}

	public static void main(String[] args) {

		try {
			final AstarCustom planner = new AstarCustom();
			planner.setHeuristic(StateHeuristic.Name.AJUSTED_SUM);
			planner.setHeuristicWeight(25);
			planner.setMaxDepth(200);
			planner.setCustomHeuristics(true);
			LOGGER.info(
				"Max depth: " + planner.getMaxDepth() + ", Custom heuristics: "
						+ planner.isCustomHeuristics() + ", Heuristics weight: "
						+ planner.getHeuristicWeight() + "\n");
			CommandLine cmd = new CommandLine(planner);
			cmd.execute(args);
		}
		catch (IllegalArgumentException e) {
			LOGGER.fatal(e.getMessage() + "\n");
		}
	}
}
