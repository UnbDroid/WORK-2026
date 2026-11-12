(define (problem advanced-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
    cubo1 cubo2 cubo3 cubo4 cubo5 cubo6 - cube
    tag1 tag2 tag3 tag4 tag5 tag6 - tag
    slot1 slot2 slot3 - slot
    slot101 slot102 slot103 slot104 slot105 slot106 - table-slot
  )
  (:init
    (robot-at start)
    (connected start w1)
    (connected w1 finish)

    (arm-empty) 
    (vision-ready) 
    (base-free)

    (cube-tag cubo1 tag1) 
    (cube-tag cubo2 tag2) 
    (cube-tag cubo3 tag3)
    (cube-tag cubo4 tag4) 
    (cube-tag cubo5 tag5) 
    (cube-tag cubo6 tag6)

    (tslot-at slot101 w1) 
    (tslot-at slot102 w1) 
    (tslot-at slot103 w1)
    (tslot-at slot104 w1) 
    (tslot-at slot105 w1) 
    (tslot-at slot106 w1)

    (cube-at cubo1 w1) 
    (cube-at cubo2 w1) 
    (cube-at cubo3 w1)
    (cube-at cubo4 w1) 
    (cube-at cubo5 w1) 
    (cube-at cubo6 w1)

    (cube-in-tslot cubo3 slot101) 
    (cube-in-tslot cubo5 slot102)
    (cube-in-tslot cubo2 slot103) 
    (cube-in-tslot cubo6 slot104)
    (cube-in-tslot cubo1 slot105) 
    (cube-in-tslot cubo4 slot106)

    (empty-slot slot1) 
    (empty-slot slot2) 
    (empty-slot slot3)

    (target cubo1 w1) 
    (target cubo2 w1) 
    (target cubo3 w1)
    (target cubo4 w1) 
    (target cubo5 w1) 
    (target cubo6 w1)
  )
  (:goal (and
    (cube-in-tslot cubo1 slot101) 
    (cube-in-tslot cubo2 slot102)
    (cube-in-tslot cubo3 slot103) 
    (cube-in-tslot cubo4 slot104)
    (cube-in-tslot cubo5 slot105) 
    (cube-in-tslot cubo6 slot106)
    
    (robot-at finish)))
)