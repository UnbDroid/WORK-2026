(define (problem advanced-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
    cubo1 cubo2 cubo3 - cube
    tag1-red tag2-blue tag3-red - tag
    slot1 slot2 slot3 - slot
    slot101 slot102 slot103 - table-slot
  )
  (:init
    (robot-at w1)
    (connected w1 finish)

    (arm-empty) 
    (vision-ready) 
    (base-free)

    (tag-cube cubo1 tag1-red) 
    (tag-cube cubo2 tag2-blue) 
    (tag-cube cubo3 tag3-red)

    (tslot-at slot101 w1) 
    (tslot-at slot102 w1) 
    (tslot-at slot103 w1)

    (cube-at cubo1 w1) 
    (cube-at cubo2 w1) 
    (cube-at cubo3 w1)

    (cube-in-tslot cubo3 slot101) 
    (cube-in-tslot cubo1 slot102)
    (cube-in-tslot cubo2 slot103)

    (empty-slot slot1) 
    (empty-slot slot2) 
    (empty-slot slot3)

    (target cubo1 w1) 
    (target cubo2 w1) 
    (target cubo3 w1)
  )
  (:goal (and
    (cube-in-tslot cubo1 slot101) 
    (cube-in-tslot cubo2 slot102)
    (cube-in-tslot cubo3 slot103)
    )
  )
)