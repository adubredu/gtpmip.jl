(define (problem pb3)
  (:domain blocksworld)
  (:objects a b c d e f g h i)
  (:init (on a b) (on b c) (on c d) (on d e) (on e f) 
         (on f g) (on g h) (on h i) (on-table i)
         (clear a)  (arm-empty))
  (:goal (and (holding i))))