; Teste: robo comeca em w6 (zona de empilhamento, tag de referencia 7),
; com cube2 no slot1 e cube3 no slot2. Ja comeca com vision-ready.
; Goal: pilha com cube3 em cima de cube2.

(define (problem teste-w6-empilhamento)
    (:domain salamander-domain)

    (:objects
        w6 - workspace
        cube2 cube3 - cube
        tag2 tag3 tag7 - tag
        slot1 slot2 slot3 - slot
    )

    (:init
        (robot-at w6)
        (arm-empty)
        (base-free)
        (vision-ready)

        (stacking-area w6)
        (tag-desk w6 tag7)

        (tag-cube cube2 tag2)
        (tag-cube cube3 tag3)

        (at-slot cube2 slot1)
        (at-slot cube3 slot2)
        (empty-slot slot3)

        (target cube2 w6)
        (target cube3 w6)
    )

    (:goal
        (and
            (cube-stacked-top cube3 cube2 w6)
        )
    )
)