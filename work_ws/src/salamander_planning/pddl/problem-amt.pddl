; Gerado por tools/yaml_to_pddl.py a partir de AMT (Advanced Manipulation Test)
; duracao da prova: 15 min
;
; Areas do YAML -> workspaces do PDDL:
;   PP_2   -> w16
;
; Cubos (tag -> cor): 1=azul, 2=azul, 3=vermelho, 4=vermelho, 5=azul, 6=vermelho
;
; Furo de destino de cada cubo (para o executor):
;   ZONA_DO_CUBO = {1: 'zone101', 2: 'zone102', 3: 'zone103', 4: 'zone104', 5: 'zone105', 6: 'zone106'}
; AVISO: PP: cubos [1, 2, 3, 4, 5, 6] ficam so com a rotina de precisao (fora do PDDL); o goal da PP e' apenas (precision-started).

(define (problem amt-instance)
    (:domain salamander-domain)

    (:objects
        start w16 finish - workspace
    )

    (:init
        (robot-at start)
        (arm-empty)
        (base-free)
        (vision-ready)

        (connected start w16)
        (connected w16 finish)
        (precision-area w16)



    )

    (:goal
        (and
            (precision-started)   ; PP_2: liga a visao da mesa de precisao
            (robot-at finish)
        )
    )
)
