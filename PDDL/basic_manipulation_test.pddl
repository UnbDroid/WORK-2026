(define (problem basic-manipulation-test)
  (:domain exercise0)

  (:objects
    robot1 - robot
    start finish w1 w2 w3 - location
    attc1 attc2 attc3 - object
  )

  (:init
    (at-robot robot1 start)
    (at-object attc1 w1)
    (at-object attc2 w1)
    (at-object attc3 w1)
  )

  (:goal
    (and
      (at-object attc1 w2)
      (at-object attc2 w2)
      (at-object attc3 w2)
      (at-robot robot1 finish)
    )
  )
)