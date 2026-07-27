(define (problem basic-manipulation-test)
  (:domain exercise0)

  (:objects
    robot1 - robot

    start target - location

    attc1 attc2 attc3 attc4 - object
  )

  (:init
    ;; posição inicial do robô
    (at-robot robot1 start)

    ;; os quatro objetos começam na área inicial
    (at-object attc1 start)
    (at-object attc2 start)
    (at-object attc3 start)
    (at-object attc4 start)
  )

  (:goal
    (and
      ;; apenas três objetos precisam ser transportados
      (at-object attc1 target)
      (at-object attc2 target)
      (at-object attc3 target)
    )
  )
)