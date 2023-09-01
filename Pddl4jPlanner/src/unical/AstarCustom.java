
package unical;
/*
 * Copyright (c) 2021 by Damien Pellier <Damien.Pellier@imag.fr>.
 *
 * This file is part of PDDL4J library.
 *
 * PDDL4J is free software: you can redistribute it and/or modify * it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * PDDL4J is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License * along with PDDL4J.  If not,
 * see <http://www.gnu.org/licenses/>
 */

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
@CommandLine.Command(name = "AstarCustom", version = "AstarCustom 1.0", description = "Solves a specified planning problem using A* search strategy.", sortOptions = false, mixinStandardHelpOptions = true, headerHeading = "Usage:%n", synopsisHeading = "%n", descriptionHeading = "%nDescription:%n%n", parameterListHeading = "%nParameters:%n", optionListHeading = "%nOptions:%n")
public class AstarCustom extends AbstractPlanner {

	private static final Logger LOGGER = LogManager.getLogger(PositiveAdjustedSum.class.getName());

	public static final String HEURISTIC_SETTING = "HEURISTIC";
	public static final StateHeuristic.Name DEFAULT_HEURISTIC = StateHeuristic.Name.FAST_FORWARD;
	public static final String WEIGHT_HEURISTIC_SETTING = "WEIGHT_HEURISTIC";
	public static final double DEFAULT_WEIGHT_HEURISTIC = 1.0;
	public static final int DEFAULT_MAX_DEPTH = 100;

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
	public Plan solve(final Problem problem) {

		LOGGER.info("* Starting A* search \n");
		// Search a solution
		final long begin = System.currentTimeMillis();
		final Plan plan = this.astar(problem);
		final long end = System.currentTimeMillis();
		// If a plan is found update the statistics of the planner
		// and log search information
		if (plan != null) {
			LOGGER.info("* A* search succeeded \n");
			this.getStatistics().setTimeToSearch(end - begin);
			this.getStatistics().setMemoryUsedToSearch(endMemory - beginMemory);
		}
		else {
			LOGGER.info("* A* search failed \n");
		}
		// Return the plan found or null if the search fails.
		return plan;
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
			return new PositiveAdjustedSum(problem);
		}
		else {
			return StateHeuristic.getInstance(this.getHeuristic(), problem);
		}
	}

	public Plan astar(Problem problem) {

		Runtime runtime = Runtime.getRuntime();
		this.beginMemory = runtime.totalMemory() - runtime.freeMemory();

		// Check if the problem is supported by the planner
		if (!this.isSupported(problem)) {
			return null;
		}

		// First we create an instance of the heuristic to use to guide the search
		final StateHeuristic heuristic = this.getHeuristics(problem);

		// We get the initial state from the planning problem
		final State init = new State(problem.getInitialState());

		// We initialize the closed list of nodes (store the nodes explored)
		final Set<Node> close = new HashSet<>();

		// We initialize the opened list to store the pending node according to function f
		final double weight = this.getHeuristicWeight();
		final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {

			@Override
			public int compare(Node n1, Node n2) {

				double f1 = weight * n1.getHeuristic() + n1.getCost();
				double f2 = weight * n2.getHeuristic() + n2.getCost();
				return Double.compare(f1, f2);
			}
		});

		// We create the root node of the tree search
		final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));

		// We add the root to the list of pending nodes
		open.add(root);
		Plan plan = null;

		// We set the timeout in ms allocated to the search
		final long timeout = this.getTimeout() * 1000;
		long time = 0;

		// We start the search
		while (!open.isEmpty() && plan == null && time < timeout) {
			// We pop the first node in the pending list open
			final Node current = open.poll();

			// Ignore the node if depth exceeded
			if (current.getDepth() > this.maxDepth) {
				continue;
			}
			close.add(current);

			// If the goal is satisfied in the current node then extract the search and return it
			if (current.satisfy(problem.getGoal())) {
				LOGGER.info("Found goal at depth " + current.getDepth() + "\n");
				this.endMemory = runtime.totalMemory() - runtime.freeMemory();
				return this.extractPlan(current, problem);
			}
			else { // Else we try to apply the actions of the problem to the current node
				for (int i = 0; i < problem.getActions().size(); i++) {
					// We get the actions of the problem
					Action a = problem.getActions().get(i);
					if (a.isApplicable(current)) {
						Node next = new Node(current);
						next.setDepth(current.getDepth() + 1);
						// We apply the effect of the action
						final List<ConditionalEffect> effects = a.getConditionalEffects();
						for (ConditionalEffect ce : effects) {
							if (current.satisfy(ce.getCondition())) {
								next.apply(ce.getEffect());
							}
						}
						// We set the new child node information
						final double g = current.getCost() + 1;
						if (!close.contains(next)) {
							next.setCost(g);
							next.setParent(current);
							next.setAction(i);
							next.setHeuristic(heuristic.estimate(next, problem.getGoal()));
							open.add(next);
						}
					}
				}
			}
		}

		// Finally, we return the search computed or null if no search was found
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
			planner.setCustomHeuristics(false);
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
