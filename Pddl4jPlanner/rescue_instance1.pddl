(define (problem rescue_inst_1)
    (:domain rescue_sys)

    (:objects
        depot loc1 - zone
        b1 b2 b3 b4 b5 - box
        p1 p2 p3 - person
        r - robot
        car - carrier
        pl1 pl2 pl3 pl4 - slot
        drugs food tools - content
    )
    (:init
        (at b1 depot)
        (at b2 depot)
        (at b3 depot)
        (at b4 depot)
        (at b5 depot)

        (at r depot)
        (at car depot)

        (at food depot)
        (at drugs depot)
        (at tools depot)

        (at p1 loc1)
        (at p2 loc1)
        (at p3 loc1)

        (person-needs-content p1 food)
        (person-needs-content p1 drugs)

        (person-needs-content p2 drugs)

        (person-needs-content p3 food)

        (free-box b1)
        (free-box b2)
        (free-box b3)
        (free-box b4)
        (free-box b5)

        (free-slot pl1)
        (free-slot pl2)
        (free-slot pl3)
        (free-slot pl4)

        (slot-of-carrier pl1 car)
        (slot-of-carrier pl2 car)
        (slot-of-carrier pl3 car)
        (slot-of-carrier pl4 car)
    )
    (:goal
        (and
            ; Person 1
            (person-has-content p1 food)
            (person-has-content p1 drugs)
            ; Person 2
            (person-has-content p2 drugs)
            ; Person 3
            (person-has-content p3 food)
        )
    )
)