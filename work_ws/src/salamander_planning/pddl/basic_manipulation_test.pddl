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
    (= (total-cost) 0)

    ; Onde o robô está e as rotas
    (robot-at start)
    (connected start w1)

    ; Estado inicial do Braço
    (arm-empty)
    (arm-at initial-position)

    ; O conhecimento prévio do mundo
    (cube-at cubo1 w1) (cube-tag cubo1 tag1) (free cubo1)
    (cube-at cubo2 w1) (cube-tag cubo2 tag2) (free cubo2)
    (cube-at cubo3 w1) (cube-tag cubo3 tag3) (free cubo3)

    ; Definição dos slots nas costas do robô
    (empty-slot slot1) (slot-position slot1 pos-s1)
    (empty-slot slot2) (slot-position slot2 pos-s2)
    (empty-slot slot3) (slot-position slot3 pos-s3)
  )

  (:goal
    (and 
      ; A exigência é preencher os 3 slots do robô
      (at-slot cubo1 slot1)
      (at-slot cubo2 slot2)
      (at-slot cubo3 slot3)
    )
  )

  (:metric minimize (total-cost))
)