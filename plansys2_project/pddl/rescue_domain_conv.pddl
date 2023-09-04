(define (domain rescue_sys_conv_plansys)
    (:requirements :negative-preconditions :typing :conditional-effects :adl :universal-preconditions :durative-actions :numeric-fluents :duration-inequalities)

    ; (:types
    ;     zone locatable slot - object
    ;     movable fixed - locatable
    ;     person - fixed
    ;     box content robot carrier - movable
    ; )

    (:types
        zone locatable slot movable fixed person box content robot carrier - object
    )

    (:functions
        (path_cost ?rob - robot)
        (content_cost ?c - content)
        (box_cost ?b - box)
        (carrier_cost ?car - carrier)
    )

    (:predicates
        (at ?obj - locatable ?zone - zone)
        (free_box ?box - box)
        (box_filled ?box - box ?cont - content)
        (box_on_carrier ?box - box ?car - carrier)
        (person_has_content ?per - person ?cont - content)
        (person_needs_content ?per - person ?cont - content)
        (free_slot ?slot - slot)
        (slot_of_carrier ?slot - slot ?car - carrier)
        (free_robot ?rob - robot)
    )

    (:durative-action put_content_in_box
        :parameters (?box - box ?cont - content ?rob - robot ?zone - zone
        )
        :duration(= ?duration 1)
        :condition (and
            (at start (free_box ?box))
            (over all (at ?cont ?zone))
            (over all (at ?box ?zone))
            (over all (at ?rob ?zone))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (free_box ?box)))
            (at end (free_robot ?rob))
            (at end (box_filled ?box ?cont))
            (at end (increase (path_cost ?rob) (content_cost ?cont)))
            (at end (increase
                    (box_cost ?box)
                    (content_cost ?cont)))
        )
    )

    (:durative-action drop_content_of_box
        :parameters (?box - box ?cont - content ?rob - robot ?per - person ?zone - zone
        )

        :duration(= ?duration 1)

        :condition (and
            (at start (box_filled ?box ?cont))
            (over all (at ?box ?zone))
            (over all (at ?rob ?zone))
            (over all (at ?per ?zone))
            (over all (person_needs_content ?per ?cont))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (box_filled ?box ?cont)))
            (at start (person_has_content ?per ?cont))
            (at end (free_robot ?rob))
            (at end (free_box ?box))
            (at end (not (person_needs_content ?per ?cont)))
            (at end (increase (path_cost ?rob) (content_cost ?cont)))
            (at end (decrease
                    (box_cost ?box)
                    (content_cost ?cont)))
        )
    )

    (:durative-action put_box_on_carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :duration(= ?duration 2)
        :condition (and
            (at start (at ?box ?zone))
            (over all (at ?car ?zone))
            (over all (at ?rob ?zone))
            (over all (slot_of_carrier ?slot ?car))
            (at start (free_slot ?slot))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (at ?box ?zone)))
            (at start (not (free_slot ?slot)))
            (at end (box_on_carrier ?box ?car))
            (at end (free_robot ?rob))
            (at end (increase (path_cost ?rob) (box_cost ?box)))
            (at end (increase (carrier_cost ?car) (box_cost ?box)))
        )
    )

    (:durative-action drop_box_of_carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :duration(= ?duration 2)
        :condition (and
            (over all (at ?car ?zone))
            (over all (at ?rob ?zone))
            (at start (box_on_carrier ?box ?car))
            (over all (slot_of_carrier ?slot ?car))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (box_on_carrier ?box ?car)))
            (at end (free_robot ?rob))
            (at end (at ?box ?zone))
            (at end (free_slot ?slot))
            (at end (increase (path_cost ?rob) (box_cost ?box)))
            (at end (decrease (carrier_cost ?car) (box_cost ?box)))
        )
    )

    (:durative-action move_robot
        :parameters (?rob - robot ?from - zone ?to - zone
        )
        :duration(= ?duration 3)
        :condition (and
            (at start (at ?rob ?from))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (at ?rob ?from)))
            (at end (free_robot ?rob))
            (at end (at ?rob ?to))
            (at end (increase (path_cost ?rob) 3))
        )
    ) 

    (:durative-action move_robot_with_carrier
        :parameters (?rob - robot ?car - carrier ?from - zone ?to - zone
        )
        :duration (= ?duration (* (carrier_cost ?car) 3))
        :condition (and
            (at start (at ?rob ?from))
            (at start (at ?car ?from))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (at ?rob ?from)))
            (at start (not (at ?car ?from)))
            (at end (free_robot ?rob))
            (at end (at ?rob ?to))
            (at end (at ?car ?to))
            (at end (increase
                    (path_cost ?rob)
                    (* (carrier_cost ?car) 3)))
        )
    )
)