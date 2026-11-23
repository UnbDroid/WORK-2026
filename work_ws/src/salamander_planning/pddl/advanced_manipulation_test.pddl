(define (problem advanced-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
  )

  (:init
    (robot-at w1)
    (arm-empty)
    (vision-ready)
    (base-free)
  )

  (:goal (and
    (precision-started)
  ))
)