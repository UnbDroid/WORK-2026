(define (problem container-test)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
    cubo1 cubo2 cubo3 cubo4 cubo5 cubo6 - cube
    tag1 tag2 tag3 tag4 tag5 tag6 - tag
    slot1 slot2 slot3 - slot
    container-red container-blue - container
  )

  (:init
    (robot-at start)
    (connected start w1)
    (connected w1 finish)

    (arm-empty)
    (arm-folded)
    (base-free)

    (at-slot cubo1 slot1)
    (at-slot cubo2 slot2)
    (at-slot cubo3 slot3)

    (cube-tag cubo1 tag1)
    (cube-tag cubo2 tag2)
    (cube-tag cubo3 tag3)
    (cube-tag cubo4 tag4)
    (cube-tag cubo5 tag5)
    (cube-tag cubo6 tag6)
   
    (free cubo4)
    (free cubo5)
    (free cubo6)

    (cube-at cubo4 w1)
    (cube-at cubo5 w1)
    (cube-at cubo6 w1)

    (target cubo1 w1)
    (target cubo2 w1)

    (container-at container-red w1)
    (container-at container-blue w1)

    (target-container cubo1 container-blue)
    (target-container cubo2 container-red)
  )

  (:goal (and
    (cube-in cubo1 container-blue)
    (cube-in cubo2 container-red)
    (robot-at finish)
    )
  )
)