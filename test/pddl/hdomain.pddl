(define (domain westbrick)
(:requirements :strips :equality)
(:predicates (clear ?obj) 
             (handempty)
             (holding ?obj)) 

(:action pick
    :parameters (?obj)
    :precondition (and  (handempty) (clear ?obj))
    :effect (and  (holding ?obj) 
                  (not (clear ?obj)) (not (handempty))))

(:action place
    :parameters (?obj)
    :precondition (and (holding ?obj) )
    :effect (and (handempty) (clear ?obj) 
                 (not (holding ?obj)))
)

(:action move
    :parameters ()
    :precondition (and (handempty))
    :effect (and (handempty))
)

(:action move_holding
    :parameters (?obj)
    :precondition (and (holding ?obj))
    :effect (and (holding ?obj))
)
)