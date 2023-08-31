(define (domain rescue_sys_relax)
    (:requirements :negative-preconditions :typing :conditional-effects)

    ; Assumptions: robot and carrier = transporter 
    (:types
        location locatable place - object
        movable fixed - locatable
        person - fixed
        box content transporter - movable
    )

    (:predicates
        (at ?obj - locatable ?loc - location)
        (box-empty ?box - box)
        (box-contains ?box - box ?c - content)
        (box-loaded-on-transporter ?box - box ?t - transporter)
        (person-has-content ?p - person ?c - content)
        (person-needs-content ?p - person ?c - content)
        (place-available ?pl - place)
        (place-of-transporter ?pl - place ?t - transporter)
    )

    (:action fill-box-with-content
        :parameters (?b - box ?c - content ?t - transporter ?loc - location
        )
        :precondition (and
            (box-empty ?b)
            (at ?c ?loc)
            (at ?b ?loc)
            (at ?t ?loc)
        )
        :effect (and
            (box-contains ?b ?c)
            (not (box-empty ?b))
        )
    )

    (:action empty-box-leaving-content
        :parameters (?b - box ?c - content ?t - transporter ?p - person ?loc - location
        )
        :precondition (and
            (box-contains ?b ?c)
            (at ?b ?loc)
            (at ?t ?loc)
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

    (:action load-one-box-on-transporter
        :parameters (?b - box ?t - transporter ?loc - location ?pl - place
        )
        :precondition (and
            (at ?b ?loc)
            (at ?t ?loc)
            (place-of-transporter ?pl ?t)
            (place-available ?pl)
        )
        :effect (and
            (not (at ?b ?loc))
            (box-loaded-on-transporter ?b ?t)
            (not (place-available ?pl))
        )
    )

    (:action unload-one-box-from-transporter
        :parameters (?b - box ?t - transporter ?loc - location ?pl - place
        )
        :precondition (and
            (at ?t ?loc)
            (box-loaded-on-transporter ?b ?t)
            (place-of-transporter ?pl ?t)
            (not (place-available ?pl))
        )
        :effect (and
            (at ?b ?loc)
            (not (box-loaded-on-transporter ?b ?t))
            (place-available ?pl)
        )
    )

    (:action move-transporter
        :parameters (?t - transporter ?from - location ?to - location
        )
        :precondition (and (at ?t ?from))
        :effect (and
            (not (at ?t ?from))
            (at ?t ?to)
        )
    )

    ; (:action move-robot-carrier
    ;     :parameters (?r - robot ?car - carrier ?from - location ?to - location
    ;     )
    ;     :precondition (and
    ;         (at ?r ?from)
    ;         (at ?car ?from)
    ;     )
    ;     :effect (and
    ;         (not (at ?r ?from))
    ;         (at ?r ?to)
    ;         (not (at ?car ?from))
    ;         (at ?car ?to)
    ;     )
    ; )
)