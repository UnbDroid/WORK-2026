(define (problem shelf-placement-test)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
    cubo1 - cube
    tag1 - tag
  )

  (:init
    (robot-at w1)

    (connected w1 finish)
    (connected finish w1)

    (arm-empty)
    (arm-folded)
    (vision-ready)
    (base-free)

    ; Cube initially on the table
    (cube-at cubo1 w1)

    ; AprilTag associated with the cube
    (tag-cube cubo1 tag1)

    (free cubo1)

    ; Shelf located at the same workspace
    (shelf w1)
  )

  (:goal
    (and
      (on-top-shelf cubo1 w1)
      (robot-at w1)
      (arm-empty)
      (arm-folded)
    )
  )
)