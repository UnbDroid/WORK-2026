(define (problem simple_stack_test)
  (:domain salamander-domain)

  (:objects
    start w1 - workspace
    cubo1 cubo2 - cube
    tag1 tag2 - tag
    slot1 slot2 slot3 - slot
    container-blue - container
  )
  
  (:init
    (robot-at w1)
    (arm-empty)
    (vision-ready)
    (base-free)

    (empty-slot slot2)
    (empty-slot slot3)
    (at-slot cubo1 slot1)
    
    (tag-cube cubo1 tag1)

    (container-at container-blue w1)
    (target-container cubo1 container-blue)
    (target cubo1 w1)

  )

  (:goal (and
    (cube-in cubo1 container-blue)
    (robot-at w1)
    )
  )
)