(define (domain rescue_robots)

    (:types
        location locatable place − object moving not−moving − locatable person − not−moving box content robot carrier − moving
    )

    (:predicates
        ( at ?obj − locatable ?loc − location)
        ( box−empty ?box − box)
        ( box−contains ?box − box ?c − content)
        ( box−loaded−on−carrier ?box − box ?car − carrier)
        ( person−has−content ?p − person ?c − content)
        ( person−needs−content ?p − person ?c − content)
        ( place−available ?pl − place)
        ( place−of−carrier ?pl − place ?car − carrier)
    )

    (:action fill−box−with−content
        :parameters ( ?b − box ?c − content ?r − robot ?loc − location
        )
        :precondition (and
            ( box−empty ?b)
            ( at ?c ?loc)
            ( at ?b ?loc)
            ( at ?r ?loc)
        ) : effects (and
            ( box−contains ?b ?c)
            (not ( box−empty ?b))
        )
    )

    ( :action fill−box−with−content
        :parameters ( ?b − box ?c − content ?r − robot ?loc − location
        )
        :precondition (and
            ( box−empty ?b)
            ( at ?c ?loc)
            ( at ?b ?loc)
            ( at ?r ?loc)
        )
        :effect (and
            ( box−contains ?b ?c)
            ( not ( box−empty ?b))
        )
    )

    ( :action empty−box−leaving−content
        :parameters ( ?b − box ?c − content ?r − robot ?p − person ?loc − location
        )
        :precondition (and
            ( box−contains ?b ?c)
            ( at ?b ?loc)
            ( at ?r ?loc)
            ( at ?p ?loc)
            ( person−needs−content ?p ?c)
        )
        :effect (and
            ( not ( box−contains ?b ?c))
            ( box−empty ?b)
            ( person−has−content ?p ?c)
        )
    )

    ( :action load−one−box−on−carrier
        :parameters ( ?b − box ?car − carrier ?r − robot ?loc − location ?pl − place
        )
        :precondition (and
            ( at ?b ?loc)
            ( at ?car ?loc)
            ( at ?r ?loc)
            ( p l a c e − o f − carrier ?pl ?car)
            ( place−av ailabl e ?pl)
        )
        :effect (and
            ( not ( at ?b ?loc))
            ( box−loaded−on−carrier ?b ?car)
            ( not ( place−av ailabl e ?pl))
        )
    )

    ( :action unload−one−box−from−carrier
        :parameters ( ?b − box ?car − carrier ?r − robot ?loc − location ?pl − place
        )
        :precondition (and
            ( at ?car ?loc)
            ( at ?r ?loc)
            ( box−loaded−on−carrier ?b ?car)
            ( p l a c e − o f − carrier ?pl ?car)
            ( not ( place−av ailabl e ?pl))
        )
        :effect (and
            ( at ?b ?loc)
            ( not ( box−loaded−on−carrier ?b ?car))
            ( place−av ailabl e ?pl)
        )
    )

    ( :action move−robot
        :parameters ( ?r − robot ?from − location ?to − location
        )
        :precondition ( at ?r ?from)
        :effect (and 6
            ( not ( at ?r ?from))
            ( at ?r ?to)
        )
    )

    ( :action move−robot−carrier
        :parameters ( ?r − robot ?car − carrier ?from − location ?to − location
        )
        :precondition (and
            ( at ?r ?from)
            ( at ?car ?from)
        )
        :effect (and
            ( not ( at ?r ?from))
            ( at ?r ?to)
            ( not ( at ?car ?from))
            ( at ?car ?to)
        )
    )

)