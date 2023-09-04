(define (domain rescue_sys_plansys)
    (:requirements :negative-preconditions :typing :conditional-effects :adl :universal-preconditions :durative-actions)

    (:types zone locatable slot person box content robot carrier )

    (:predicates
        (robot_at ?rob - robot ?zone - zone)
        (box_at ?box - box ?zone - zone)
        (carrier_at ?car - carrier ?zone - zone)
        (content_at ?cont - content ?zone - zone)
        (person_at ?per - person ?zone - zone)
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
            (over all (content_at ?cont ?zone))
            (over all (box_at ?box ?zone))
            (over all (robot_at ?rob ?zone))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (free_box ?box)))
            (at end (free_robot ?rob))
            (at end (box_filled ?box ?cont))
        )
    )

    (:durative-action drop_content_of_box
        :parameters (?box - box ?cont - content ?rob - robot ?per - person ?zone - zone
        )

        :duration(= ?duration 1)

        :condition (and
            (at start (box_filled ?box ?cont))
            (over all (box_at ?box ?zone))
            (over all (robot_at ?rob ?zone))
            (over all (person_at ?per ?zone))
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
        )
    )

    (:durative-action put_box_on_carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :duration(= ?duration 2)
        :condition (and
            (at start (box_at ?box ?zone))
            (over all (carrier_at ?car ?zone))
            (over all (robot_at ?rob ?zone))
            (over all (slot_of_carrier ?slot ?car))
            (at start (free_slot ?slot))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (box_at ?box ?zone)))
            (at start (not (free_slot ?slot)))
            (at end (box_on_carrier ?box ?car))
            (at end (free_robot ?rob))
        )
    )

    (:durative-action drop_box_of_carrier
        :parameters (?box - box ?car - carrier ?rob - robot ?zone - zone ?slot - slot
        )
        :duration(= ?duration 2)
        :condition (and
            (over all (carrier_at ?car ?zone))
            (over all (robot_at ?rob ?zone))
            (at start (box_on_carrier ?box ?car))
            (over all (slot_of_carrier ?slot ?car))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (box_on_carrier ?box ?car)))
            (at end (free_robot ?rob))
            (at end (box_at ?box ?zone))
            (at end (free_slot ?slot))
        )
    )

    (:durative-action move_robot
        :parameters (?rob - robot ?from - zone ?to - zone
        )
        :duration(= ?duration 3)
        :condition (and
            (at start (robot_at ?rob ?from))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (robot_at ?rob ?from)))
            (at end (free_robot ?rob))
            (at end (robot_at ?rob ?to))
        )
    ) 

    (:durative-action move_robot_with_carrier
        :parameters (?rob - robot ?car - carrier ?from - zone ?to - zone
        )
        :duration(= ?duration 3)
        :condition (and
            (at start (robot_at ?rob ?from))
            (at start (carrier_at ?car ?from))
            (at start (free_robot ?rob))
        )
        :effect (and
            (at start (not (free_robot ?rob)))
            (at start (not (robot_at ?rob ?from)))
            (at start (not (carrier_at ?car ?from)))
            (at end (free_robot ?rob))
            (at end (robot_at ?rob ?to))
            (at end (carrier_at ?car ?to))
        )
    )
)