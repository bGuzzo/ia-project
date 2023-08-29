(define (problem rescue_inst_2)
    (:domain rescue_sys)
    (:requirements :disjunctive-preconditions)

    (:objects
        depot loc1 loc2 loc3 loc4 loc5 - location
        b1 b2 b3 - box
        p1 p2 p3 p4 p5 p6 - person
        r1 r2 - robot
        carl car2 - carrier
        pl1car1 pl2car1 pl1car2 pl2car2 - place
        drugs food tools - content
    )
    (:init
        (at b1 depot)
        (at b2 depot)
        (at b3 depot)

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

        (box-empty b1)
        (box-empty b2)
        (box-empty b3)

        (place-available pl1car1)
        (place-available pl2car1)
        (place-available pl1car2)
        (place-available pl2car2)

        (place-of-carrier pl1car1 carl)
        (place-of-carrier pl2car1 carl)
        (place-of-carrier pl1car2 car2)
        (place-of-carrier pl2car2 car2)
    )
    (:goal
        (and
            (or
                (person-has-content p1 food)
            )

            (or
                (person-has-content p1 tools)
            )

            (person-has-content p2 drugs)
            (person-has-content p3 drugs)

            (person-has-content p4 food)
            (person-has-content p4 drugs)

            (person-has-content p5 food)
            (person-has-content p5 drugs)
            (person-has-content p5 tools)

            (person-has-content p6 food)
            (person-has-content p6 drugs)
            (person-has-content p6 tools)
        )
    )
)