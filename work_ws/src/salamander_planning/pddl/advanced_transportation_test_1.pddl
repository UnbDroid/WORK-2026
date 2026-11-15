(define (problem simple_stack_test)
  (:domain salamander-domain)

  (:objects
    start w1 w2 w3 w4 w5 w6 finish - workspace
    cubo1 cubo2 cubo3 cubo4 cubo5 cubo6 - cube
    tag1 tag2 tag3 tag4 tag5 tag6 - tag
    tag_desk_w5 - tag
    slot1 slot2 slot3 slot_fantasma - slot
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
    (target-container cubo2 container-blue)

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
    (cube-in cubo2 container-blue)
    (cube-at cubo3 w4)
    (cube_stacked cubo5 cubo4 w5)
    (cube-at cubo6 w6)
    (robot-at finish)
    )
  )
)