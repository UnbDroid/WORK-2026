(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 - workspace
    
    cubo1 cubo2 cubo3 - cube
    tag1 - tag
    slot1 slot2 slot3 - slot
    pos-s1 pos-s2 pos-s3 - arm-position
  )

  (:init
    ; Onde o robô está
    (robot-at start)
    ; (connected start w1)

    ; Estado inicial do Braço
    (arm-empty)
    (arm-at initial-position)

    ; O conhecimento prévio do mundo
    (cube-at cubo1 start)
    (cube-tag cubo1 tag1)
    (free cubo1)
    
    ; (cube-at cubo2 w1) (cube-tag cubo2 tag2) (free cubo2)
    ; (cube-at cubo3 w1) (cube-tag cubo3 tag3) (free cubo3)

    ; Definição dos slots nas costas do robô
    (empty-slot slot1) (slot-position slot1 pos-s1)
    (empty-slot slot2) (slot-position slot2 pos-s2)
    (empty-slot slot3) (slot-position slot3 pos-s3)

    (aligned-tag tag1 start)
  )

  (:goal
    (and 
      ; A exigência é preencher o slot com o cubo
      (at-slot cubo1 slot1)
    )
  )
)