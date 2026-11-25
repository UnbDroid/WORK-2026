(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 w3 finish - workspace
    cubo1 cubo2 cubo3 cubo4 - cube
    tag3 tag4 tag19 tag25 - tag
    slot1 slot2 slot3 - slot
    red blue - color
  )

  (:init

    (robot-at start)
    (connected start w1) (connected start w3)
    (connected w1 w3) (connected w3 w1)
    (connected w3 finish) (connected w1 finish)
	
    (cube-color cubo1 blue)
    (cube-color cubo2 blue)
    (cube-color cubo3 blue)
    (cube-color cubo4 blue)
    (tag-cube cubo1 tag3)
    (tag-cube cubo2 tag4)
    (tag-cube cubo3 tag19)
    (tag-cube cubo4 tag25)


    (arm-empty)
    (vision-ready)
    (base-free)

    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-at cubo3 w1)
    (cube-at cubo4 w1)

    (free cubo1)
    (free cubo2)
    (free cubo3)
    (free cubo4)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)



    (target cubo1 w3)
    (target cubo2 w3)
    (target cubo3 w3)
  )

  (:goal
    (and 
      (cube-at cubo1 w3)
      (cube-at cubo2 w3)
      (cube-at cubo3 w3)

      (robot-at finish)
    )
  )
  
)
