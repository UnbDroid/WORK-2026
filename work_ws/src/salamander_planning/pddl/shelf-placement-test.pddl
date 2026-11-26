(define (problem advanced_transportation_test_1)
(:domain salamander-domain)

(:objects
  start w1 w2 w4 w5 w6 w7 w8 w9 w10 w11 w12 w13 w14 w15 finish - workspace
  cubo1 cubo2 cubo3 - cube
  tag4 tag9 tag25 tag7 - tag
  slot1 slot2 slot3 - slot
  container-blue - container
  red blue - color
)

(:init
  (robot-at start)

  (connected start w7)
  (connected w7 w8)
  (connected w8 w10)
  (connected w10 w6)
  (connected w6 w4)



  (arm-empty)
  (vision-ready)
  (base-free)

  (empty-slot slot1)
  (empty-slot slot2)
  (empty-slot slot3)

  (cube-color cubo1 red)
  (cube-color cubo2 blue)
  (cube-color cubo3 red)

  (tag-cube cubo1 tag4)
  (tag-cube cubo2 tag9)
  (tag-cube cubo3 tag25)

  ; (shelf w5)

  (cube-at cubo1 w8) (free cubo1)
  (cube-at cubo2 w8) (free cubo2)
  (cube-at cubo3 w8) (free cubo3)

  ;(stacking-area w2)
  ;(tag-desk w2 tag7)

  ; (container-at container-blue w3)
  ; (target-container cubo3 container-blue)

  (target cubo1 w10)
  (target cubo2 w10)
  (target cubo3 w10)
)

(:goal (and
  (robot-at w6)
  (cube-at cubo1 w10)
  (cube-at cubo2 w10)
  (cube-at cubo3 w10)
  )
)
)
