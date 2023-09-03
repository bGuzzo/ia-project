(define (problem rescue_inst_2)
    (:domain rescue_sys)
    (:requirements :disjunctive-preconditions)

    (:objects
        depot zone1 zone2 zone3 zone4 zone5 - zone
        b1 b2 b3 - box
        p1 p2 p3 p4 p5 p6 - person
        r1 r2 - robot
        carl car2 - carrier
        sl1car1 sl2car1 sl1car2 sl2car2 - slot
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

        (at p1 zone1)
        (at p2 zone1)
        (at p3 zone2)
        (at p4 zone3)
        (at p5 zone4)
        (at p6 zone5)

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
        
        (free-box b1)
        (free-box b2)
        (free-box b3)

        (free-slot sl1car1)
        (free-slot sl2car1)
        (free-slot sl1car2)
        (free-slot sl2car2)

        (slot-of-carrier sl1car1 carl)
        (slot-of-carrier sl2car1 carl)
        (slot-of-carrier sl1car2 car2)
        (slot-of-carrier sl2car2 car2)
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
        )
    )
)