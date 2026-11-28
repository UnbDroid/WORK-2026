(define (problem advanced_transportation_test_1)
(:domain salamander-domain)

(:objects
  start w1 w2 w4 w5 w6 w7 w8 w9 w10 w11 w12 w13 w14 w15 finish - workspace
  cubo1 cubo2 - cube
  tag16 tag30 tag200 - tag
  slot1 slot2 slot3 - slot
  ;container-blue - container
  red blue - color
)

(:init
  (robot-at start)

  (connected start w9)
  (connected w9 w6)

  (connected w6 finish)



  (arm-empty)
  (vision-ready)
  (base-free)

  (empty-slot slot1)
  (empty-slot slot2)
  (empty-slot slot3)

  (cube-color cubo1 red)
  (cube-color cubo2 blue)

  (tag-cube cubo1 tag16)
  (tag-cube cubo2 tag30)

  ; (shelf w5)

  (cube-at cubo1 w9) (free cubo1)
  (cube-at cubo2 w9) (free cubo2)

  (stacking-area w6)
  (tag-desk w6 tag200)

  ; (container-at container-blue w3)
  ; (target-container cubo3 container-blue)

  (target cubo1 w9)
  (target cubo2 w9)
)

(:goal (and
  (robot-at finish)
  (cube-at cubo1 w6)
  (cube-at cubo2 w6)
  )
)
)
