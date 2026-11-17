(define (problem simple_stack_test)
  (:domain salamander-domain)

  (:objects
    start w1 w2 w3 finish - workspace
    cubo1 cubo2 cubo3 - cube

    tag1-red tag2-blue - tag
    slot1 slot2 slot3 slot_fantasma - slot
    container-blue - container
  )

  (:init
    (robot-at start)
    (connected start w1) (connected w1 start)
    (connected start w2) (connected w2 start)
    (connected start w3) (connected w3 start)

    (connected w1 w2) (connected w2 w1)
    (connected w1 w3) (connected w3 w1)
    (connected w1 finish) (connected finish w1)

    (connected w2 w3) (connected w3 w2)
    (connected w2 finish) (connected finish w2)

    (connected w3 finish) (connected finish w3)

    (arm-empty)
    (vision-ready)
    (base-free)


    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-at cubo3 w3)
    (free cubo1)
    (free cubo2)

    (tag-cube cubo1 tag1-red)
    (tag-cube cubo2 tag2-blue)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)
    
    (container-at container-blue w2)

    (target-container cubo1 container-blue)
    (target-container cubo2 container-blue)
  )

  (:goal (and
    (cube-in cubo1 container-blue)
    (cube-in cubo2 container-blue)
    
    (cube-at cubo3 w2)
    (robot-at finish)
    )
  )
)