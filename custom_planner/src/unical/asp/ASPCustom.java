package unical.asp;

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
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import picocli.CommandLine;

import unical.asp.Node;

@CommandLine.Command(name = "ASPCustom", version = "ASPCustom 1.0", description = "Solves a specified planning problem using A* search strategy.", sortOptions = false, mixinStandardHelpOptions = true, headerHeading = "Usage:%n", synopsisHeading = "%n", descriptionHeading = "%nDescription:%n%n", parameterListHeading = "%nParameters:%n", optionListHeading = "%nOptions:%n")
public class ASPCustom extends AbstractPlanner {

    private double heuristicWeight;

    private StateHeuristic.Name heuristic;

    @CommandLine.Option(names = { "-w",
            "--weight" }, defaultValue = "1.0", paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    @CommandLine.Option(names = { "-e",
            "--heuristic" }, defaultValue = "FAST_FORWARD", description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
                    + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }

    private static final Logger LOGGER = LogManager.getLogger(ASPCustom.class.getName());

    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    // @Override
    // public Plan solve(final Problem problem) {
    // // Creates the A* search strategy
    // StateSpaceSearch search =
    // StateSpaceSearch.getInstance(SearchStrategy.Name.ASTAR,
    // this.getHeuristic(), this.getHeuristicWeight(), this.getTimeout());
    // LOGGER.info("* Starting A* search \n");
    // // Search a solution
    // Plan plan = search.searchPlan(problem);
    // // If a plan is found update the statistics of the planner and log search
    // information
    // if (plan != null) {
    // LOGGER.info("* A* search succeeded\n");
    // this.getStatistics().setTimeToSearch(search.getSearchingTime());
    // this.getStatistics().setMemoryUsedToSearch(search.getMemoryUsed());
    // } else {
    // LOGGER.info("* A* search failed\n");
    // }
    // // Return the plan found or null if the search fails.
    // return plan;
    // }

    @Override
    public Plan solve(final Problem problem) {
        final long begin = System.currentTimeMillis();
        Plan plan = null;
        try {
            plan = this.astar(problem);
        } catch (ProblemNotSupportedException e) {
            LOGGER.error("Error: "+e.getMessage());
            e.printStackTrace();
        }
        final long end = System.currentTimeMillis();
        if (plan != null) {
            LOGGER.info("Plan found");
            this.getStatistics().setTimeToSearch(end - begin);
        } else {
            LOGGER.warn("Plan NOT found");
        }
        return plan;
    }

    /**
     * Search a solution plan for a planning problem using an A* search strategy.
     *
     * @param problem the problem to solve.
     * @return a plan solution for the problem or null if there is no solution
     * @throws ProblemNotSupportedException if the problem to solve is not supported
     *                                      by the planner.
     */
    public Plan astar(Problem problem) throws ProblemNotSupportedException {
        // Check if the problem is supported by the planner
        if (!this.isSupported(problem)) {
            throw new ProblemNotSupportedException("Problem not supported");
        }

        // First we create an instance of the heuristic to use to guide the search
        final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);

        // We get the initial state from the planning problem
        final State init = new State(problem.getInitialState());

        // We initialize the closed list of nodes (store the nodes explored)
        final Set<Node> close = new HashSet<>();

        // We initialize the opened list to store the pending node according to function
        final double weight = this.getHeuristicWeight();
        final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {
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
        final int timeout = this.getTimeout() * 1000;
        long time = 0;

        // We start the search
        while (!open.isEmpty() && plan == null && time < timeout) {

            // We pop the first node in the pending list open
            final Node current = open.poll();
            close.add(current);

            // If the goal is satisfied in the current node then extract the search and
            // return it
            if (current.satisfy(problem.getGoal())) {
                return this.extractPlan(current, problem);
            } else { // Else we try to apply the actions of the problem to the current node
                for (int i = 0; i < problem.getActions().size(); i++) {
                    // We get the actions of the problem
                    Action a = problem.getActions().get(i);
                    // If the action is applicable in the current node
                    if (a.isApplicable(current)) {
                        Node next = new Node(current);
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
        return plan;
    }

    /**
     * Extracts a search from a specified node.
     *
     * @param node    the node.
     * @param problem the problem.
     * @return the search extracted from the specified node.
     */
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

    public static void main(String[] args) {
        try {
            final ASPCustom planner = new ASPCustom();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }
    }

    /**
     * Returns if a specified problem is supported by the planner. Just ADL problem
     * can be solved by this planner.
     *
     * @param problem the problem to test.
     * @return <code>true</code> if the problem is supported <code>false</code>
     *         otherwise.
     */
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
                || problem.getRequirements().contains(RequireKey.HIERARCHY))
                        ? false
                        : true;
    }
}