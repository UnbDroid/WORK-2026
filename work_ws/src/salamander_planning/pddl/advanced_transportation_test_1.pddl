(define (problem advanced-transportation-test-1)
  (:domain salamander-domain)

  (:objects
    start w1 w2 w3 w4 w5 w11 w12 finish - workspace
  )

  (:init
    (robot-at start)
    (connected start w1) 
    (connected w1 w2)
    (connected w2 w11)
    (connected w11 w3)
    (connected w3 w4)
    (connected w4 w12) 
    (connected w12 w5)
    (connected w5 finish)

    (arm-empty)
    (vision-ready)
    (base-free)

  )

  (:goal
    (and 
      (robot-at finish)
    )
  )
)
