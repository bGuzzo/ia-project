# ia-project

This repository contains the code developed for the Classical AI exam at UNICAL (University of Calabria). The project focuses on planning in Artificial Intelligence, using the PDDL (Planning Domain Definition Language) to model a rescue scenario.

## Project Description

The project involves modeling a rescue scenario where a robot has to manage resources and deliver them to people in different locations. The robot can move, carry a carrier with boxes, load/unload contents to boxes, and load/unload boxes to the carrier. The goal is to develop a planner that can efficiently determine the sequence of actions the robot needs to take to satisfy the needs of all the people.

## Repository Structure

The repository is organized as follows:

```
ia-project/
├── Pddl4jPlanner
├── pddl_files
├── plansys2_project
├── planutils
└── README.md
```

### Directory Description

* **pddl\_files:** Contains the PDDL domain and problem instance files used for the classical planning part of the project. [cite: 252]
   
* **Pddl4jPlanner:** An Eclipse project using the PDDL4J library. It includes the implementation of a custom planner (`AstarCustom.java`) and a custom heuristic (`PositiveAdjustedSum.java`). [cite: 253, 254]
   
* **plansys2\_project:** A ROS2 workspace for the PlanSys2 project, containing all the necessary files to build and run the ROS2 package, including the modified PDDL domain. This directory is used for the robotics planning part of the project. [cite: 255, 256]
   
* **planutils:** Contains files and instructions for the temporal planning part of the project. [cite: 257]

## Classical Planning

### Planner

* The planner implemented in this project uses the A\* algorithm, combining Greedy and UCS strategies. [cite: 50]
   
* The search is ordered by a function f(x) = w \* h(x) + g(x), where w is a multiplicative factor for the heuristic (set to 25 in this project), h(x) is the heuristic function, and g(x) is the cost function. [cite: 51]
   
* A depth limit is set to 200 to prevent memory exhaustion, but this limit is not reached in the solutions. [cite: 52]
   
* The cost function g(x) is the number of nodes explored (depth). [cite: 53]
   
* The planner is implemented by extending the `AbstractPlanner` class from the PDDL4J library with the `AstarCustom` class. [cite: 54]
   
* The `Node` class, extending PDDL4J's `State` class, represents a node in the search tree and contains information like cost, depth, and the action that generated it. [cite: 77, 78]

### Heuristic

* A modified version of the Adjusted Sum heuristic from the PDDL4J library is used. [cite: 81, 82]
   
* The heuristic considers only positive effects (a relaxed version of the problem). [cite: 82]
   
* The heuristic function is calculated as:
   
    * h(x) = sumValue(x) + (level - maxValue()) if the goal is reachable from the current state. [cite: 83]
       
    * Infinity, otherwise. [cite: 83]
       
* Components of the heuristic function:
   
    * **Sum value:** Sum of counts of positive fluents in the relaxed planning graph that also appear in the goal. [cite: 84, 85, 86]
       
    * **Max value:** Maximum number of times a positive fluent appears in the relaxed planning graph and the goal. [cite: 87, 88, 89]
       
    * **Level:** Connectivity level of the relaxed planning graph. [cite: 89, 90]
       
* The `PositiveAdjustedSum` class implements this custom heuristic. [cite: 50, 51, 52, 53, 54, 91]

### Instances

* Three problem instances are defined in PDDL. [cite: 107, 108, 109, 110, 111, 112, 120, 121, 122, 123, 137, 138, 139]
   
    * **Instance 1:** 5 boxes, 3 people (p1, p2, p3) with specific needs (p1: food, drugs; p2: drugs; p3: food), 1 robot, 1 carrier (capacity 4). [cite: 107, 108, 109, 110, 111, 112]
       
    * **Instance 2:** 2 robots, 2 carriers (capacity 2), 3 boxes, 6 people (p1-p6) with various needs (p1: food or tools; p2, p3, p4: medicine; p4: also food; p5, p6: all). Note: The exclusive condition for p1's needs (food or tools) could not be implemented due to PDDL4J limitations. [cite: 120, 121, 122, 123, 124, 125, 126, 127]
       
    * **Instance 3 (Bonus):** 2 robots, 2 carriers (capacity 2), 4 boxes, 8 people (p1-p8) with needs similar to Instance 2. Similar to Instance 2, the exclusive condition for p1 is replaced with an AND condition.
       
* The planner solves Instance 1 in 1.9 seconds using 56.66 MB of memory. [cite: 117, 118, 119]
   
* Instance 2 requires more resources, taking about 30 seconds and 250 MB. [cite: 131, 132, 133, 134, 135, 136]
   
* Instance 3 requires significantly more resources: approximately 3.5 minutes and 1 GB of memory. [cite: 145, 146, 147, 148, 149, 150, 151, 152]
   
* The implemented planner and heuristic show good efficiency compared to others in the PDDL4J library. [cite: 153, 154, 155]

## Temporal Planning

* The domain and Instance 1 are converted to durative actions to model the problem temporally. [cite: 157]
   
* Weights are introduced for each content type (food, medicine, tools) to affect the cost of moving the carrier. [cite: 158]
   
* Functions are added to define the total cost of operations (`path-cost`), content weight (`content-cost`), box weight (`box-cost`), and carrier weight (`carrier-cost`). [cite: 159, 160]
   
* A predicate `free-robot` is added to track the robot's availability. [cite: 160, 161, 162, 163, 164, 165]
   
* Actions are converted to durative actions with costs and effects at the start and end. [cite: 162, 163, 164, 165]
   
* Action durations and costs are defined. [cite: 164, 165]
   
    * Loading/unloading content: duration 1, cost = content weight. [cite: 164, 165, 166, 167]
       
    * Loading/unloading box: duration 2, cost = box weight. [cite: 165, 167, 168, 169]
       
    * Moving robot: duration and cost 3. [cite: 165, 169, 170]
       
    * Moving robot with carrier: duration and cost = carrier cost \* 3. [cite: 165, 170, 171]
       

* The first instance of the problem is modified to incorporate the changes, set content weights, and specify the objective to minimize the overall path cost. [cite: 171, 172, 173]

* The planutils library with the lpg-td planner is used to generate a plan for the temporal problem.

## Robotics Planning

* The plan generated in the previous section is integrated into a robotic software architecture using the PlanSys2 package. [cite: 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 200, 201, 466]

* A new PlanSys2 package is created. [cite: 186]

* The actions of the domain are redefined as fake-actions (C++ files). [cite: 186, 195, 196, 197, 198, 199, 200]
   
* The domain file is modified to be compatible with PlanSys2 (removing type hierarchy, variable action durations, and cost calculations). [cite: 186, 187, 188, 189, 200, 201, 481, 482, 483, 484, 485, 486, 487]
   
* The first instance of the problem is converted to operations compatible with PlanSys2. [cite: 189, 190, 191, 208, 209, 210]
   
* A Python file is implemented to describe the package launch and map to the fake-actions. [cite: 192, 211, 212, 213, 214]
   
* Dependencies and build operations are defined in a CMake file. [cite: 193, 214, 215, 216]
   
* Additional package information (maintainer, license) is in the package.xml file. [cite: 194, 216, 217, 218]

* The package is compiled, and the plan is executed in the PlanSys2 terminal. [cite: 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247]

* The plan is applied correctly and respects the temporal constraints. [cite: 245, 246, 247]

## Deliverables

The delivered files are organized in the following directory structure:

```
ia-project/
├── Pddl4jPlanner
├── pddl_files
├── plansys2_project
├── planutils
└── README.md
```

The structure and meaning of the directories are as follows:

1.  **pddl\_files:** Contains the domain and problem instances for the first and second parts of the project. [cite: 252, 253, 254, 255, 256, 257]
   
2.  **Pddl4jPlanner:** An Eclipse project using the PDDL4J library, containing a planner (`AstarCustom.java`) and heuristic (`PositiveAdjustedSum.java`). [cite: 253, 254, 334]
   
    * To execute, import the project in Eclipse and run `AstarCustom`, passing the domain, problem file, and timeout (ms) as arguments (e.g., `rescue_domain.pddl rescue_instance3.pddl -t 999999`). [cite: 254, 255]
       
3.  **plansys2\_project:** PlanSys2 workspace, containing files for building and running the package, including the modified domain. [cite: 255, 256] This directory is for the robotics planning part of the project. [cite: 466]
   
4.  **planutils:** Contains instructions, domain, and instance for the temporal planning part of the project. [cite: 257, 437, 438, 439, 453, 454, 455]