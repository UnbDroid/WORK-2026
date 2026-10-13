(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 - workspace
    cubo1 cubo2 cubo3 - cube
    tag1 tag2 tag3 - tag
    slot1 slot2 slot3 - slot
    pos-s1 pos-s2 pos-s3 - arm-position
  )

  (:init
    ; Onde o robô está
    (robot-at start)
    (connected start w1)

    ; Estado inicial do Braço
    (arm-empty)
    (arm-at initial-position)

    ; O conhecimento prévio do mundo
    (cube-at cubo1 w1)
    (cube-at cubo2 w1)
    (cube-tag cubo1 tag1)
    (cube-tag cubo2 tag2)
    
    ; Ambos os cubos precisam estar livres para a garra pegar!
    (free cubo1)
    (free cubo2)

    ; Definição dos slots nas costas do robô
    (empty-slot slot1) (slot-position slot1 pos-s1)
    (empty-slot slot2) (slot-position slot2 pos-s2)
    (empty-slot slot3) (slot-position slot3 pos-s3)
  )

  (:goal
    (and 
      ; A exigência é preencher os slots 1 e 2 com os respectivos cubos
      (at-slot cubo1 slot1)
      (at-slot cubo2 slot2)
    )
  )
)