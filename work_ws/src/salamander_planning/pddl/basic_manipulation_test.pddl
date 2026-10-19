(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
    cubo1 cubo2 cubo3 - cube
    tag1 tag2 tag3 - tag
    slot1 slot2 slot3 - slot
  )

  (:init
    (robot-at start)
    (connected start w1)
    (connected w1 finish)

    (arm-empty)
    (arm-folded)
    (base-free)

    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-at cubo3 w1)
    (cube-tag cubo1 tag1)
    (cube-tag cubo2 tag2)
    (cube-tag cubo3 tag3)
    (free cubo1)
    (free cubo2)
    (free cubo3)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)
  )

  (:goal
    (and 
      (cube-at cubo1 finish)
      (cube-at cubo2 finish)
      (cube-at cubo3 finish)
    )
  )
)