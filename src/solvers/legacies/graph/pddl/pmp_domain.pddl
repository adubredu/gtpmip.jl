(define (domain pmp)
    (:requirements :strips )
    (:types
        objs 
        locs
    )
    (:predicates
        (robot-at ?loc - locs)
        (object-at ?ob - objs ?loc - locs)
        (clear ?ob - objs)
        (hand-empty)
        (holding ?ob - objs)
        (on ?top - objs ?bot - objs)
    )

    (:action pick
        :parameters (?ob - objs  ?oloc - locs)
        :precondition (and  (hand-empty) (clear ?ob) (robot-at ?oloc) 
                      (object-at ?ob ?oloc))
        :effect (and (holding ?ob) (not (clear ?ob)) (not (hand-empty)) 
                (not (object-at ?ob ?oloc)))
    )

    (:action place
        :parameters (?ob - objs  ?oloc - locs)
        :precondition (and (holding ?ob) (robot-at ?oloc) 
                      (not (object-at ?ob ?oloc)))
        :effect (and (hand-empty) (not (holding ?ob)) (clear ?ob) 
                (object-at ?ob ?oloc))
    )

    (:action stack
        :parameters (?top - objs ?bot - objs ?rloc - locs)
        :precondition (and (clear ?bot) (holding ?top) (robot-at ?rloc) (object-at ?bot ?rloc))
        :effect (and (on ?top ?bot) (hand-empty) (not (holding ?top)) 
                (not (clear ?bot)) (object-at ?top ?rloc))
    )

    (:action unstack
        :parameters (?top - objs ?bot - objs ?rloc - locs)
        :precondition (and (on ?top ?bot) (hand-empty) (clear ?top) 
                      (robot-at ?rloc) (object-at ?top ?rloc))
        :effect (and (not (on ?top ?bot)) (holding ?top) (not (clear ?top)) 
                (not (hand-empty)) (clear ?bot))
    )
    

    (:action move
        :parameters (?loc1 - locs ?loc2 - locs)
        :precondition (and (robot-at ?loc1) (not (robot-at ?loc2)) (hand-empty))
        :effect (and (robot-at ?loc2) (not (robot-at ?loc1)))
    )
    
    (:action move-holding
        :parameters (?obj - objs ?loc1 - locs ?loc2 - locs)
        :precondition (and (holding ?obj) (not (hand-empty))  (robot-at ?loc1) (not (robot-at ?loc2)) )
        :effect (and (robot-at ?loc2) (not (robot-at ?loc1)) )
    )
)
    