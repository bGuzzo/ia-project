(define (domain rescue_sys)
    (:requirements :negative-preconditions :typing :conditional-effects)

    (:types
        location locatable place - object
        movable fixed - locatable
        person - fixed
        box content robot carrier - movable
    )

    (:predicates
        (at ?obj - locatable ?loc - location)
        (box-empty ?box - box)
        (box-contains ?box - box ?c - content)
        (box-loaded-on-carrier ?box - box ?car - carrier)
        (person-has-content ?p - person ?c - content)
        (person-needs-content ?p - person ?c - content)
        (place-available ?pl - place)
        (place-of-carrier ?pl - place ?car - carrier)
    )

    (:action fill-box-with-content
        :parameters (?b - box ?c - content ?r - robot ?loc - location
        )
        :precondition (and
            (box-empty ?b)
            (at ?c ?loc)
            (at ?b ?loc)
            (at ?r ?loc)
        )
        :effect (and
            (box-contains ?b ?c)
            (not (box-empty ?b))
        )
    )

    (:action empty-box-leaving-content
        :parameters (?b - box ?c - content ?r - robot ?p - person ?loc - location
        )
        :precondition (and
            (box-contains ?b ?c)
            (at ?b ?loc)
            (at ?r ?loc)
            (at ?p ?loc)
            (person-needs-content ?p ?c)
        )
        :effect (and
            (not (box-contains ?b ?c))
            (box-empty ?b)
            (person-has-content ?p ?c)
            (not (person-needs-content ?p ?c))
        )
    )

    (:action load-one-box-on-carrier
        :parameters (?b - box ?car - carrier ?r - robot ?loc - location ?pl - place
        )
        :precondition (and
            (at ?b ?loc)
            (at ?car ?loc)
            (at ?r ?loc)
            (place-of-carrier ?pl ?car)
            (place-available ?pl)
        )
        :effect (and
            (not (at ?b ?loc))
            (box-loaded-on-carrier ?b ?car)
            (not (place-available ?pl))
        )
    )

    (:action unload-one-box-from-carrier
        :parameters (?b - box ?car - carrier ?r - robot ?loc - location ?pl - place
        )
        :precondition (and
            (at ?car ?loc)
            (at ?r ?loc)
            (box-loaded-on-carrier ?b ?car)
            (place-of-carrier ?pl ?car)
            (not (place-available ?pl))
        )
        :effect (and
            (at ?b ?loc)
            (not (box-loaded-on-carrier ?b ?car))
            (place-available ?pl)
        )
    )

    (:action move-robot
        :parameters (?r - robot ?from - location ?to - location
        )
        :precondition (and (at ?r ?from))
        :effect (and
            (not (at ?r ?from))
            (at ?r ?to)
        )
    )

    (:action move-robot-carrier
        :parameters (?r - robot ?car - carrier ?from - location ?to - location
        )
        :precondition (and
            (at ?r ?from)
            (at ?car ?from)
        )
        :effect (and
            (not (at ?r ?from))
            (at ?r ?to)
            (not (at ?car ?from))
            (at ?car ?to)
        )
    )
)