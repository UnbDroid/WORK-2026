(define (problem basic-manipulation-test)
  (:domain exercise0)

  (:objects
    robot1 - robot
    start finish w1 - location

    ; w1 e w2 tirados por enquanto
    ; attc1 attc2 attc3 - object
  )

  (:init
    (at-robot robot1 start)
    ; (arm-empty)
    ; (at-object attc1 w1)
    ; (at-object attc2 w1)
    ; (at-object attc3 w1)

    (connected start w1)
    (connected w1 finish)
  )

  (:goal
    (and 
    ; (at-object attc1 w2)
    ; (at-object attc2 w2)
    ; (at-object attc3 w2)

    (visited w1) ;o robo deve passar por aq
    (at-robot robot1 finish) ;robo tem q terminar aq    
    )
  )
)