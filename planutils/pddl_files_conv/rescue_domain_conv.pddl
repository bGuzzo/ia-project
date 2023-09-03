(define (domain rescue_sys_conv)
    (:requirements :negative-preconditions :typing :conditional-effects :adl :universal-preconditions :durative-actions :numeric-fluents :duration-inequalities)

    (:types
        zone locatable slot - object
        movable fixed - locatable
        person - fixed
        box content robot carrier - movable
    )

    (:functions
        (path-cost ?rob - robot)
        (content-cost ?c - content)
        (box-cost ?b - box)
        (carrier-cost ?car - carrier)
    )

    (:predicates
        (at ?obj - locatable ?zone - zone)
        (free-box ?box - box)
        (box-filled ?box - box ?cont - content)
        (box-on-carrier ?box - box ?car - carrier)
        (person-has-content ?per - person ?cont - content)
        (person-needs-content ?per - person ?cont - content)
        (free-slot ?slot - slot)
        (slot-of-carrier ?slot - slot ?car - carrier)
        (free-robot ?rob - robot)
    )

    (:durative-action put-content-in-box
        :parameters (?box - box ?cont - content ?rob - robot ?zone - zone
        )
        :duration(= ?duration 1)
        :condition (and
            (at start (free-box ?box))
            (over all (at ?cont ?zone))
            (over all (at ?box ?zone))
            (over all (at ?rob ?zone))
            (at start (free-robot ?rob))
        )
        :effect (and
            (at start (not (free-robot ?rob)))
            (at start (not (free-box ?box)))
            (at end (free-robot ?rob))
            (at end (box-filled ?box ?cont))
            (at end (increase (path-cost ?rob) (content-cost ?cont)))
            (at end (increase
                    (box-cost ?box)
                    (content-cost ?cont)))
        )
    )

    (:durative-action drop-content-of-box
        :parameters (?box - box ?cont - content ?rob - robot ?per - person ?zone - zone
        )

        :duration(= ?duration 1)

        :condition (and
            (at start (box-filled ?box ?cont))
            (over all (at ?box ?zone))
            (over all (at ?rob ?zone))
            (over all (at ?per ?zone))
            (over all (person-needs-content ?per ?cont))
            (at start (free-robot ?rob))
        )
        :effect (and
            (at start (not (free-robot ?rob)))
            (at start (not (box-filled ?box ?cont)))
            (at start (person-has-content ?per ?cont))
            (at end (free-robot ?rob))
            (at end (free-box ?box))
            (at end (increase (path-cost ?rob) (content-cost ?cont)))
            (at end (not (person-needs-content ?per ?cont)))
            (at end (decrease
                    (box-cost ?box)
                    (content-cost ?cont)))
        )
    )

    (:durative-action put-box-on-carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :duration(= ?duration 2)
        :condition (and
            (at start (at ?box ?zone))
            (over all (at ?car ?zone))
            (over all (at ?rob ?zone))
            (over all (slot-of-carrier ?slot ?car))
            (at start (free-slot ?slot))
            (at start (free-robot ?rob))
        )
        :effect (and
            (at start (not (free-robot ?rob)))
            (at start (not (at ?box ?zone)))
            (at start (not (free-slot ?slot)))
            (at end (box-on-carrier ?box ?car))
            (at end (free-robot ?rob))
            (at end (increase (path-cost ?rob) (box-cost ?box)))
            (at end (increase (carrier-cost ?car) (box-cost ?box)))
        )
    )

    (:durative-action drop-box-of-carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :duration(= ?duration 2)
        :condition (and
            (over all (at ?car ?zone))
            (over all (at ?rob ?zone))
            (at start (box-on-carrier ?box ?car))
            (over all (slot-of-carrier ?slot ?car))
            (at start (free-robot ?rob))
        )
        :effect (and
            (at start (not (free-robot ?rob)))
            (at start (not (box-on-carrier ?box ?car)))
            (at end (free-robot ?rob))
            (at end (at ?box ?zone))
            (at end (free-slot ?slot))
            (at end (increase (path-cost ?rob) (box-cost ?box)))
            (at end (decrease (carrier-cost ?car) (box-cost ?box)))
        )
    )

    (:durative-action move-robot
        :parameters (?rob - robot ?from - zone ?to - zone
        )
        :duration(= ?duration 3)
        :condition (and
            (at start (at ?rob ?from))
            (at start (free-robot ?rob))
        )
        :effect (and
            (at start (not (free-robot ?rob)))
            (at start (not (at ?rob ?from)))
            (at end (free-robot ?rob))
            (at end (at ?rob ?to))
            (at end (increase (path-cost ?rob) 3))
        )
    ) 

    (:durative-action move-robot-with-carrier
        :parameters (?rob - robot ?car - carrier ?from - zone ?to - zone
        )
        :duration (= ?duration (* (carrier-cost ?car) 3))
        :condition (and
            (at start (at ?rob ?from))
            (at start (at ?car ?from))
            (at start (free-robot ?rob))
        )
        :effect (and
            (at start (not (free-robot ?rob)))
            (at start (not (at ?rob ?from)))
            (at start (not (at ?car ?from)))
            (at end (free-robot ?rob))
            (at end (at ?rob ?to))
            (at end (at ?car ?to))
            (at end (increase
                    (path-cost ?rob)
                    (* (carrier-cost ?car) 3)))
        )
    )
)