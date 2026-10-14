(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
    cubo1 cubo2 cubo3 - cube
    tag1 tag2 tag3 - tag
    slot1 slot2 slot3 - slot
    pos-s1 pos-s2 pos-s3 - arm-position
  )

  (:init
    ; Onde o robô está
    (robot-at start)
    (connected start w1)
    (connected w1 finish)

    ; Estado inicial do robo
    (arm-empty)
    (arm-at initial-position)
    (base-free)

    ; O conhecimento prévio do mundo
    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-at cubo3 w1)
    (cube-tag cubo1 tag1)
    (cube-tag cubo2 tag2)
    (cube-tag cubo3 tag3)
    
    ; Ambos os cubos precisam estar livres para a garra pegar!
    (free cubo1)
    (free cubo2)
    (free cubo3)

    ; Definição dos slots nas costas do robô
    (empty-slot slot1) (slot-position slot1 pos-s1)
    (empty-slot slot2) (slot-position slot2 pos-s2)
    (empty-slot slot3) (slot-position slot3 pos-s3)
  )

  (:goal
    (and 
      (cube-at cubo1 finish)
      (cube-at cubo2 finish)
      (cube-at cubo3 finish)
    )
  )
)