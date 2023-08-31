(define (problem rescue_inst_2_relax_2)
    (:domain rescue_sys_relax_2)
    (:requirements :disjunctive-preconditions)

    (:objects
        depot loc1 loc2 loc3 loc4 loc5 - location
        s1 s2 s3 s4 - slot
        ;s1 s2 - slot
        p1 p2 p3 p4 p5 p6 - person
        t1 t2 - transporter
        ;t1 - transporter
        ; pl1car1 pl2car1 pl1car2 pl2car2 - place
        ; pl1tra1 pl2tra1 pl1tra2 pl2tra2 - place
        drugs food tools - content
    )
    (:init

        (at t1 depot)
        (at t2 depot)

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

        (slot-empty s1)
        (slot-empty s2)
        (slot-empty s3)
        (slot-empty s4)

        (slot-of-transporter s1 t1)
        (slot-of-transporter s2 t1)
        ;(slot-of-transporter s2 t2)
        (slot-of-transporter s3 t2)
        (slot-of-transporter s4 t2)
    )
    (:goal
        (and
            ;(or
            (person-has-content p1 food)
            ;)

            ;(or
            (person-has-content p1 tools)
            ;)

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