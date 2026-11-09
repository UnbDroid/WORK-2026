(define (problem basic-transportation-test-2)
  (:domain salamander-domain)

  (:objects
    start w1 w2 w3 w4 finish - workspace
    cubo1 cubo2 cubo3 cubo4 cubo5 - cube
    tag1 tag2 tag3 tag4 tag5 - tag
    slot1 slot2 slot3 - slot
  )

  (:init
    (robot-at start)
    
    (connected start w1) (connected w1 start)
    (connected start w2) (connected w2 start)
    (connected start w3) (connected w3 start)
    (connected start w4) (connected w4 start)

    (connected w1 w2) (connected w2 w1)
    (connected w1 w3) (connected w3 w1)
    (connected w1 w4) (connected w4 w1)
    
    (connected w2 w3) (connected w3 w2)
    (connected w2 w4) (connected w4 w2)
    
    (connected w3 w4) (connected w4 w3)

    (connected w1 finish)
    (connected w2 finish)
    (connected w3 finish)
    (connected w4 finish)

    (arm-empty)
    (vision-ready)
    (base-free)

    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-at cubo3 w2)
    (cube-at cubo4 w3)
    (cube-at cubo5 w4)
    
    (tag_cube cubo1 tag1)
    (tag_cube cubo2 tag2)
    (tag_cube cubo3 tag3)
    (tag_cube cubo4 tag4)
    (tag_cube cubo5 tag5)
    
    (free cubo1)
    (free cubo2)
    (free cubo3)
    (free cubo4)
    (free cubo5)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)

    (target cubo1 w3)
    (target cubo2 w4)
    (target cubo3 w1)
    (target cubo4 w2)
    (target cubo5 w2)
  )

  (:goal
    (and 
      (cube-at cubo1 w3)
      (cube-at cubo2 w4)
      (cube-at cubo3 w1)
      (cube-at cubo4 w2)
      (cube-at cubo5 w2)

      (robot-at finish)
    )
  )
)