(define (problem rescue_inst_3)
    (:domain rescue_sys)
    (:requirements :disjunctive-preconditions)

    (:objects
        depot loc1 loc2 loc3 loc4 loc5 loc6 loc7 - zone
        b1 b2 b3 b4 - box
        p1 p2 p3 p4 p5 p6 p7 p8 - person
        r1 r2 - robot
        carl car2 - carrier
        pl1car1 pl2car1 pl1car2 pl2car2 - slot
        drugs food tools - content
    )
    (:init
        (at b1 depot)
        (at b2 depot)
        (at b3 depot)
        (at b4 depot)

        (at r1 depot)
        (at r2 depot)
        (at carl depot)
        (at car2 depot)

        (at food depot)
        (at drugs depot)
        (at tools depot)

        (at p1 loc1)
        (at p2 loc1)
        (at p3 loc2)
        (at p4 loc3)
        (at p5 loc4)
        (at p6 loc5)
        (at p7 loc6)
        (at p8 loc7)

        (person-needs-content p1 food)
        (person-needs-content p1 tools)

        (person-needs-content p2 drugs)

        (person-needs-content p3 drugs)

        (person-needs-content p4 food)
        (person-needs-content p4 drugs)

        (person-needs-content p5 food)
        (person-needs-content p5 drugs)
        (person-needs-content p5 tools)

        (person-needs-content p6 food)
        (person-needs-content p6 drugs)
        (person-needs-content p6 tools)

        (person-needs-content p7 food)
        (person-needs-content p7 drugs)
        (person-needs-content p7 tools)

        (person-needs-content p8 food)
        (person-needs-content p8 drugs)
        (person-needs-content p8 tools)

        (free-box b1)
        (free-box b2)
        (free-box b3)
        (free-box b4)

        (free-slot pl1car1)
        (free-slot pl2car1)
        (free-slot pl1car2)
        (free-slot pl2car2)

        (slot-of-carrier pl1car1 carl)
        (slot-of-carrier pl2car1 carl)
        (slot-of-carrier pl1car2 car2)
        (slot-of-carrier pl2car2 car2)
    )
    (:goal
        (and
            ; Person 1
            (person-has-content p1 food)
            (person-has-content p1 tools)
            ; Person 2
            (person-has-content p2 drugs)
            ; Person 3
            (person-has-content p3 drugs)
            ; Person 4
            (person-has-content p4 food)
            (person-has-content p4 drugs)
            ; Person 5
            (person-has-content p5 food)
            (person-has-content p5 drugs)
            (person-has-content p5 tools)
            ; Person 6
            (person-has-content p6 food)
            (person-has-content p6 drugs)
            (person-has-content p6 tools)
            ; Person 7
            (person-has-content p7 food)
            (person-has-content p7 drugs)
            (person-has-content p7 tools)
            ; Person 8
            (person-has-content p8 food)
            (person-has-content p8 drugs)
            (person-has-content p8 tools)
        )
    )
)