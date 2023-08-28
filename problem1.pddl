(define (problem deliveringcontents)
    (:domain emergencyServicesLogistics)

    (:objects
        depot loc1 - location
        b1 b2 b3 b4 b5 - box
        p1 p2 p3 - person
        r - robot
        car - carrier
        pl1 pl2 pl3 pl4 - place
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

        (box-empty b1)
        (box-empty b2)
        (box-empty b3)
        (box-empty b4)
        (box-empty b5)

        (place-available pl1)
        (place-available pl2)
        (place-available pl3)
        (place-available pl4)

        (place-of-carrier pl1 car)
        (place-of-carrier pl2 car)
        (place-of-carrier pl3 car)
        (place-of-carrier pl4 car)
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