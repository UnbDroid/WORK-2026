(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w6 w7 finish - workspace
    cubo1 cubo2 cubo3 cubo4 - cube
    tag1 tag2 tag3 tag4 - tag
    slot1 slot2 slot3 - slot
  )

  (:init

    (robot-at start)
    (connected start w6)
    (connected w6 w7)
    (connected w7 finish)

    (arm-empty)
    (vision-ready)
    (base-free)

    (cube-at cubo1 w6)
    (cube-at cubo2 w6)
    (cube-at cubo3 w6)
    (tag-cube cubo1 tag1)
    (tag-cube cubo2 tag2)
    (tag-cube cubo3 tag3)
    (free cubo1)
    (free cubo2)
    (free cubo3)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)

    (target cubo1 w7)
    (target cubo2 w7)
    (target cubo3 w7)
  )

  (:goal
    (and 
      (cube-at cubo1 w7)
      (cube-at cubo2 w7)
      (cube-at cubo3 w7)

      (robot-at finish)
    )
  )
  
)
