(define (problem advanced_transportation_test_1)
(:domain salamander-domain)

(:objects
  start w2 w3 w4 finish - workspace
  cubo1 cubo2 cubo3 - cube
  tag1 tag2 tag3 tag7 - tag
  slot1 slot2 slot3 - slot
  container-blue - container
  red blue - color
)

(:init
  (robot-at start)

  (connected start w4)
  (connected w4 w2)
  (connected w2 w3)
  (connected w3 finish)

  (arm-empty)
  (vision-ready)
  (base-free)

  (empty-slot slot1)
  (empty-slot slot2)
  (empty-slot slot3)

  (cube-color cubo1 red)
  (cube-color cubo2 blue)
  (cube-color cubo3 red)

  (tag-cube cubo1 tag1)
  (tag-cube cubo2 tag2)
  (tag-cube cubo3 tag3)

  ; (shelf w5)

  (cube-at cubo1 w4) (free cubo1)
  (cube-at cubo2 w4) (free cubo2)
  (cube-at cubo3 w3) (free cubo3)

  (stacking-area w2)
  (tag-desk w2 tag7)

  (container-at container-blue w3)
  (target-container cubo3 container-blue)

  (target cubo1 w2)
  (target cubo2 w2)
  (target cubo3 w3)
)

(:goal (and
  (cube-stacked-top cubo2 cubo1 w2)
  (cube-in cubo3 container-blue)
  (robot-at finish)
  )
)
)