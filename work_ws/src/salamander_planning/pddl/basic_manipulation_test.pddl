(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 w4 finish - workspace
    cubo1 cubo2 cubo3 cubo4 - cube
    tag1-red tag2-blue tag3-red - tag
    slot1 slot2 slot3 - slot
  )

  (:init

    (robot-at start)
    (connected start w1)
    (connected w1 w4)
    (connected w4 finish)

    (arm-empty)
    (vision-ready)
    (base-free)

    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-at cubo3 w1)
    (tag-cube cubo1 tag1-red)
    (tag-cube cubo2 tag2-blue)
    (tag-cube cubo3 tag3-red)
    (free cubo1)
    (free cubo2)
    (free cubo3)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)

    (target cubo1 w4)
    (target cubo2 w4)
    (target cubo3 w4)
  )

  (:goal
    (and 
      (cube-at cubo1 w4)
      (cube-at cubo2 w4)
      (cube-at cubo3 w4)

      (robot-at finish)
    )
  )
  
)