(define (problem btt1-task)
  (:domain salamander-domain)

  (:objects
    start w1 finish - workspace
    cube01 cube03 cube05 cube06 - cube
    tag01 tag03 tag05 tag06 - tag
    slot1 slot2 slot3 - slot
    pos-slot1 pos-slot2 pos-slot3 - arm-position
    level0 level1 level2 - height
  )

  (:init
    (connected start w1)
    (connected w1 start)
    (connected w1 finish)
    (connected finish w1)
    (connected start finish)
    (connected finish start)

    (robot-at start)
    (arm-empty)
    (arm-at initial-position)

    (empty-slot slot1)
    (slot-position slot1 pos-slot1)
    
    (empty-slot slot2)
    (slot-position slot2 pos-slot2)
    
    (empty-slot slot3)
    (slot-position slot3 pos-slot3)

    (cube-at cube01 w1)
    (cube-tag cube01 tag01)
    (free cube01)

    (cube-at cube03 w1)
    (cube-tag cube03 tag03)
    (free cube03)

    (cube-at cube05 w1)
    (cube-tag cube05 tag05)
    (free cube05)

    (cube-at cube06 w1)
    (cube-tag cube06 tag06)
    (free cube06)

    (next-height level0 level1)
    (next-height level1 level2)

    (=(total-cost) 0)
  )

  (:goal
    (and
      (cube-at cube01 finish)
      (cube-at cube03 finish)
      (cube-at cube05 finish)
      (cube-at cube06 finish)
    )
  )

  (:metric minimize (total-cost))
)