(define (problem rescue_inst_1_relax_2)
    (:domain rescue_sys_relax_2)

    (:objects
        depot loc1 - location
        s1 s2 s3 s4 s5 - slot
        p1 p2 p3 - person
        t - transporter
        drugs food tools - content
    )
    (:init
        (at t depot)

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

        (slot-empty s1)
        (slot-empty s2)
        (slot-empty s3)
        (slot-empty s4)
        (slot-empty s5)

        (slot-of-transporter s1 t)
        (slot-of-transporter s2 t)
        (slot-of-transporter s3 t)
        (slot-of-transporter s4 t)
        (slot-of-transporter s5 t)
    )
    (:goal
        (and
            (person-has-content p1 food)
            (person-has-content p1 drugs)
            (person-has-content p2 drugs)
            (person-has-content p3 food)
        )
    )
)