(define (problem problem1) (:domain pmp)
(:objects  
    red - objs
    blue - objs 
    green - objs
    A - locs 
    B - locs
    C - locs
)

(:init
    (object-at red A)
    (object-at blue A)
    (object-at green B)
    (on red blue)
    (hand-empty)
    (robot-at C)
    (clear red)
    (clear green)
)

(:goal (and
    (object-at blue B)
    ;(robot-at B)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
