(define (problem advanced_transportation_test_1)
  (:domain salamander-domain)

  (:objects
    start w1 w2 w3 w4 w5 w6 finish - workspace
    cubo1 cubo2 cubo3 cubo4 cubo5 cubo6 - cube
    tag1 tag2 tag3 tag4 tag5 tag6 - tag
    tag_desk_w5 - tag
    slot1 slot2 slot3 slot_fantasma - slot
    container-red container-blue - container
  )

  (:init
    (robot-at start)
    (connected start w1) 
    (connected w1 w2) (connected w1 w3) (connected w1 w4) (connected w1 w5) (connected w1 w6) 
    (connected w2 w3) (connected w2 w4) (connected w2 w5) (connected w2 w6)
    (connected w3 w4) (connected w3 w5) (connected w3 w6)
    (connected w4 w5) (connected w5 w6)
    (connected w6 finish)

    (arm-empty)
    (vision-ready)
    (base-free)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)

    (slot-enabled slot1) 
    (enables slot1 slot2)
    (enables slot2 slot3)
    (enables slot3 slot_fantasma)

    (tag_cube cubo1 tag1)
    (tag_cube cubo2 tag2)
    (tag_cube cubo3 tag3)
    (tag_cube cubo4 tag4)
    (tag_cube cubo5 tag5)
    (tag_cube cubo6 tag6)

    (cube-at cubo1 w1) (free cubo1)
    (cube-at cubo2 w1) (free cubo2)
    (cube-at cubo3 w1) (free cubo3)
    (cube-at cubo4 w4) (free cubo4)
    (cube-at cubo5 w4) (free cubo5)
    (cube-at cubo6 w6) (free cubo6)

    (container-at container-blue w3)
    (container-at container-red w2)
    (target-container cubo1 container-blue)
    (target-container cubo2 container-red)

    (stacking-area w5)
    (tag_desk w5 tag_desk_w5)

    (target cubo1 w3)
    (target cubo2 w2)
    (target cubo3 w4)
    (target cubo4 w5)
    (target cubo5 w5)
    (target cubo6 w6)
  )

  (:goal (and
    (cube-in cubo1 container-blue)
    (cube-in cubo2 container-red)
    (cube-at cubo3 w4)
    (cube_stacked cubo5 cubo4 w5)
    (cube-at cubo6 w6)
    (robot-at finish)
    )
  )
)