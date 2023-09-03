(define (problem rescue_inst_1)
    (:domain rescue_sys)

    (:objects
        depot zone1 zone2 - zone
        b1 b2 b3 b4 b5 - box
        p1 p2 p3 - person
        r - robot
        car - carrier
        sl1 sl2 sl3 sl4 - slot
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

        (at p1 zone1)
        (at p2 zone1)
        (at p3 zone2)

        (person-needs-content p1 food)
        (person-needs-content p1 drugs)

        (person-needs-content p2 drugs)

        (person-needs-content p3 food)

        (free-box b1)
        (free-box b2)
        (free-box b3)
        (free-box b4)
        (free-box b5)

        (free-slot sl1)
        (free-slot sl2)
        (free-slot sl3)
        (free-slot sl4)

        (slot-of-carrier sl1 car)
        (slot-of-carrier sl2 car)
        (slot-of-carrier sl3 car)
        (slot-of-carrier sl4 car)
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