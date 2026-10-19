(define (domain salamander-domain)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    workspace
    cube
    tag
    slot
    height
  )

  (:predicates
    (robot-at ?w - workspace)
    (connected ?w1 - workspace ?w2 - workspace)
    
    ;; Estados Funcionais e de Segurança do Braço
    (arm-empty)
    (holding ?c - cube)
    (arm-folded)    ;; Braço recolhido (Seguro para mover a base)
    (vision-ready)  ;; Braço em zero_position (Câmera consegue enxergar)

    (cube-at ?c - cube ?w - workspace)
    (cube-tag ?c - cube ?t - tag)
    (free ?c - cube)

    (empty-slot ?s - slot)
    (at-slot ?c - cube ?s - slot)

    (tag-found ?t - tag ?w - workspace)
    (container-found ?w - workspace)
    
    (aligned-tag ?t - tag ?w - workspace)
    (container-aligned ?w - workspace)

    (precision-slot ?t - tag ?w - workspace)
    (stacking-area ?w - workspace)
    (on-top ?c-top - cube ?c-bottom - cube)
    (cube-height ?c - cube ?h - height)
    (next-height ?h1 - height ?h2 - height)

    (base-free)
  )

  (:action move
    :parameters (?origin - workspace ?destination - workspace)
    :precondition (and
      (robot-at ?origin)
      (connected ?origin ?destination)
      (arm-folded) 
      (arm-empty)  
    )
    :effect (and
      (not (robot-at ?origin))
      (robot-at ?destination)
    )
  )

  ;; CORREÇÃO: Adicionado ?w para o solver não ignorar a ação
  (:action zero_position
    :parameters (?w - workspace)
    :precondition (and (robot-at ?w) (arm-folded) (arm-empty))
    :effect (and (not (arm-folded)) (vision-ready))
  )

  ;; CORREÇÃO: Adicionado ?w para o solver não ignorar a ação
  (:action reset_arm
    :parameters (?w - workspace)
    :precondition (and (robot-at ?w) (vision-ready) (arm-empty))
    :effect (and (not (vision-ready)) (arm-folded))
  )

  (:action find-container
    :parameters (?w - workspace)
    :precondition (robot-at ?w)
    :effect (and (container-found ?w))
  )

  (:action align_to_cube
    :parameters (?w - workspace ?c - cube ?t - tag)
    :precondition (and 
      (robot-at ?w) 
      (cube-at ?c ?w) 
      (cube-tag ?c ?t)
      (base-free)
      (arm-empty)
      (vision-ready)
    )
    :effect (and 
      (aligned-tag ?t ?w)
      (not (base-free))
    )
  )

  (:action align-tag
    :parameters (?w - workspace ?t - tag)
    :precondition (and (robot-at ?w) (arm-empty) (tag-found ?t ?w))
    :effect (and (aligned-tag ?t ?w))
  )

  (:action align-container
    :parameters (?w - workspace)
    :precondition (and (robot-at ?w) (container-found ?w))
    :effect (and (container-aligned ?w))
  )

  (:action get_cube_slot
    :parameters (?c - cube ?s - slot ?w - workspace)
    :precondition (and
      (robot-at ?w)
      (arm-empty)
      (at-slot ?c ?s)
      (arm-folded)
    )
    :effect (and
      (not (arm-empty))
      (holding ?c)
      (not (at-slot ?c ?s))
      (empty-slot ?s)
      (not (arm-folded))
      (vision-ready)
    )
  )

  (:action put_cube_table
    :parameters (?w - workspace ?c - cube)
    :precondition (and 
      (robot-at ?w) 
      (holding ?c) 
      (vision-ready)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (free ?c)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action put-cube-container
    :parameters (?w - workspace ?c - cube)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (container-aligned ?w)
      (vision-ready)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action put-cube-shelf
    :parameters (?w - workspace ?c - cube)
    :precondition (and (robot-at ?w) (holding ?c) (vision-ready))
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action put-cube-precision
    :parameters (?w - workspace ?c - cube ?t-hole - tag)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (aligned-tag ?t-hole ?w)
      (precision-slot ?t-hole ?w)
      (vision-ready)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action put-cube-pile
    :parameters (?w - workspace ?c-held - cube ?c-base - cube ?t-base - tag ?alt-base - height ?alt-new - height)
    :precondition (and
      (robot-at ?w)
      (stacking-area ?w)
      (holding ?c-held)
      (aligned-tag ?t-base ?w)
      (cube-tag ?c-base ?t-base)
      (cube-at ?c-base ?w)
      (free ?c-base)
      (cube-height ?c-base ?alt-base)
      (next-height ?alt-base ?alt-new)
      (vision-ready)
    )
    :effect (and
      (not (holding ?c-held))
      (arm-empty)
      (cube-at ?c-held ?w)
      (on-top ?c-held ?c-base)
      (not (free ?c-base))
      (free ?c-held)
      (cube-height ?c-held ?alt-new)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action get_cube_table
    :parameters (?w - workspace ?c - cube ?t - tag)
    :precondition (and
      (robot-at ?w)
      (arm-empty)
      (aligned-tag ?t ?w)
      (cube-tag ?c ?t)
      (cube-at ?c ?w)
      (free ?c)
      (vision-ready)
    )
    :effect (and
      (not (arm-empty))
      (holding ?c)
      (not (cube-at ?c ?w))
      (not (aligned-tag ?t ?w))
      (base-free)
      (not (vision-ready))
      (arm-folded)
    )
  )

  ;; CORREÇÃO: Adicionado ?w para garantir grounding perfeito e segurança
  (:action put_cube_slot
    :parameters (?c - cube ?s - slot ?w - workspace)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (empty-slot ?s)
      (arm-folded)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (at-slot ?c ?s)
      (not (empty-slot ?s))
      (not (arm-folded))
      (vision-ready)
    )
  )
)