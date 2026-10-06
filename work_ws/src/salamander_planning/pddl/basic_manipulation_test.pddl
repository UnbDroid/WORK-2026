(define (problem basic-manipulation-test)
  (:domain salamander-domain)

  (:objects
    start w1 - workspace
    cubo1 - cube
    tag1 - tag
  )

  (:init
    ; Onde o robô está e as rotas
    (robot-at start)
    (connected start w1)

    ; O conhecimento prévio do mundo (O cubo 1 está na mesa w1 e tem a tag1)
    (cube-at cubo1 w1)
    (cube-tag cubo1 tag1)
  )

  (:goal
    (and 
      ; A única exigência para o teste ter sucesso é o robô estar alinhado com a tag!
      (aligned-tag tag1 w1) 
    )
  )
)