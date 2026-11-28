; Gerado por tools/yaml_to_pddl.py a partir de BTT2 (Basic Transportation Test II)
; duracao da prova: 10 min
;
; Areas do YAML -> workspaces do PDDL:
;   WS_2   -> w2
;   WS_3   -> w3
;   WS_6   -> w6
;   WS_7   -> w7
;
; Cubos (tag -> cor): 3=vermelho, 4=vermelho, 14=vermelho, 15=vermelho, 23=vermelho
; AVISO: ha cubos indo e voltando entre workspaces (ciclo): nao existe ordem unica, liberando conexoes nos dois sentidos entre os workspaces de trabalho.

(define (problem btt2-instance)
    (:domain salamander-domain)

    (:objects
        start w2 w3 w6 w7 finish - workspace
        cube3 cube4 cube14 cube15 cube23 - cube
        tag3 tag4 tag14 tag15 tag23 - tag
        slot1 slot2 slot3 - slot
    )

    (:init
        (robot-at start)
        (arm-empty)
        (base-free)
        (vision-ready)

        (connected start w2)
        (connected w2 w3)
        (connected w2 w6)
        (connected w2 w7)
        (connected w3 w2)
        (connected w3 w6)
        (connected w3 w7)
        (connected w6 w2)
        (connected w6 w3)
        (connected w6 w7)
        (connected w7 w2)
        (connected w7 w3)
        (connected w7 w6)
        (connected w7 finish)

        (empty-slot slot1)
        (empty-slot slot2)
        (empty-slot slot3)


        (tag-cube cube3 tag3)
        (cube-at cube3 w7)
        (free cube3)
        (tag-cube cube4 tag4)
        (cube-at cube4 w3)
        (free cube4)
        (tag-cube cube14 tag14)
        (cube-at cube14 w2)
        (free cube14)
        (tag-cube cube15 tag15)
        (cube-at cube15 w6)
        (free cube15)
        (tag-cube cube23 tag23)
        (cube-at cube23 w7)
        (free cube23)

        (target cube3 w6)
        (target cube4 w7)
        (target cube14 w3)
        (target cube15 w2)
        (target cube23 w3)
    )

    (:goal
        (and
            (cube-at cube3 w6)   ; WS_6
            (cube-at cube4 w7)   ; WS_7
            (cube-at cube14 w3)   ; WS_3
            (cube-at cube15 w2)   ; WS_2
            (cube-at cube23 w3)   ; WS_3
            (robot-at finish)
        )
    )
)
