(define (domain rescue_sys)
    (:requirements :negative-preconditions :typing :conditional-effects)

    (:types
        zone locatable slot - object
        movable fixed - locatable
        person - fixed
        box content robot carrier - movable
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
    )

    (:action put-content-in-box
        :parameters (?box - box ?cont - content ?rob - robot ?zone - zone
        )
        :precondition (and
            (free-box ?box)
            (at ?cont ?zone)
            (at ?box ?zone)
            (at ?rob ?zone)
        )
        :effect (and
            (box-filled ?box ?cont)
            (not (free-box ?box))
        )
    )

    (:action drop-content-of-box
        :parameters (?box - box ?cont - content ?rob - robot ?per - person ?zone - zone
        )
        :precondition (and
            (box-filled ?box ?cont)
            (at ?box ?zone)
            (at ?rob ?zone)
            (at ?per ?zone)
            (person-needs-content ?per ?cont)
        )
        :effect (and
            (not (box-filled ?box ?cont))
            (free-box ?box)
            (person-has-content ?per ?cont)
            (not (person-needs-content ?per ?cont))
        )
    )

    (:action put-box-on-carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :precondition (and
            (at ?box ?zone)
            (at ?car ?zone)
            (at ?rob ?zone)
            (slot-of-carrier ?slot ?car)
            (free-slot ?slot)
        )
        :effect (and
            (not (at ?box ?zone))
            (box-on-carrier ?box ?car)
            (not (free-slot ?slot))
        )
    )

    (:action drop-box-of-carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :precondition (and
            (at ?car ?zone)
            (at ?rob ?zone)
            (box-on-carrier ?box ?car)
            (slot-of-carrier ?slot ?car)
            (not (free-slot ?slot))
        )
        :effect (and
            (at ?box ?zone)
            (not (box-on-carrier ?box ?car))
            (free-slot ?slot)
        )
    )

    (:action move-robot
        :parameters (?rob - robot ?from - zone ?to - zone
        )
        :precondition (and (at ?rob ?from))
        :effect (and
            (not (at ?rob ?from))
            (at ?rob ?to)
        )
    )

    (:action move-robot-with-carrier
        :parameters (?rob - robot ?car - carrier ?from - zone ?to - zone
        )
        :precondition (and
            (at ?rob ?from)
            (at ?car ?from)
        )
        :effect (and
            (not (at ?rob ?from))
            (at ?rob ?to)
            (not (at ?car ?from))
            (at ?car ?to)
        )
    )
)