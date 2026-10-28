(define (problem basic-transportation-test-1)
  (:domain salamander-domain)

  (:objects
    start w1 w2 w3 finish - workspace
    cubo1 cubo2 cubo3 - cube
    tag1 tag2 tag3 - tag
    slot1 slot2 slot3 - slot
  )

  (:init
    (robot-at start)
    (connected start w1) (connected w1 start)
    (connected start w2) (connected w2 start)
    (connected start w3) (connected w3 start)

    (connected w1 w2) (connected w2 w1)
    (connected w1 w3) (connected w3 w1)
    (connected w1 finish) (connected finish w1)

    (connected w2 w3) (connected w3 w2)
    (connected w2 finish) (connected finish w2)

    (connected w3 finish) (connected finish w3)

    (arm-empty)
    (vision-ready)
    (base-free)

    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-at cubo3 w3)
    (cube-tag cubo1 tag1)
    (cube-tag cubo2 tag2)
    (cube-tag cubo3 tag3)
    (free cubo1)
    (free cubo2)
    (free cubo3)

    (empty-slot slot1)
    (empty-slot slot2)
    (empty-slot slot3)

    ;; LÓGICA DE DESTINOS: O robô só desbloqueia a mochila nessas mesas
    (target cubo1 w3)
    (target cubo2 w2)
    (target cubo3 w2)
  )

  (:goal
    (and 
      (cube-at cubo1 w3)
      (cube-at cubo2 w2)
      (cube-at cubo3 w2)
      (robot-at finish)
    )
  )
)