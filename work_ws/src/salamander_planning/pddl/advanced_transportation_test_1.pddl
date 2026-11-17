(define (problem advanced_transportation_test_1)
(:domain salamander-domain)

(:objects
  start w1 w2 w3 w4 w5 finish - workspace
  cubo1 cubo2 cubo3 cubo4 cubo5 - cube
  tag1 tag2 tag3 tag4 tag5 tag7 - tag      
  slot1 slot2 slot3 - slot
  container-blue - container
)

(:init
  (robot-at start)

  (connected start w1) (connected w1 start)
  (connected w1 w2) (connected w2 w1)
  (connected w1 w3) (connected w3 w1)
  (connected w1 w4) (connected w4 w1)
  (connected w1 w5) (connected w5 w1)
  (connected w2 w3) (connected w3 w2)
  (connected w2 w4) (connected w4 w2)
  (connected w2 w5) (connected w5 w2)
  (connected w3 w4) (connected w4 w3)
  (connected w3 w5) (connected w5 w3)
  (connected w4 w5) (connected w5 w4)
  (connected w5 finish) (connected finish w5)

  (arm-empty)
  (vision-ready)
  (base-free)

  (empty-slot slot1)
  (empty-slot slot2)
  (empty-slot slot3)

  (tag-cube cubo1 tag1)
  (tag-cube cubo2 tag2)
  (tag-cube cubo3 tag3)
  (tag-cube cubo4 tag4)
  (tag-cube cubo5 tag5)

  (cube-at cubo1 w1) (free cubo1)
  (cube-at cubo2 w2) (free cubo2)
  (cube-at cubo3 w4) (free cubo3)
  (cube-at cubo4 w4) (free cubo4)
  (cube-at cubo5 w4) (free cubo5)

  (container-at container-blue w3)
  (target-container cubo1 container-blue)

  (stacking-area w5)
  (tag-desk w5 tag7)

  (target cubo1 w3)
  (target cubo2 w4)
  (target cubo3 w5)
  (target cubo4 w5)
  (target cubo5 w5)
)

(:goal (and
  (cube-in cubo1 container-blue)
  (cube-at cubo2 w4)
  (cube-at cubo3 w5)
  (cube-stacked-top cubo5 cubo4 w5)
  (robot-at finish)
  )
)
)