(define (domain salamander-domain)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    workspace
    cube
    tag
    slot
    container
  )

  (:predicates
    (robot-at ?w - workspace)
    (connected ?w1 - workspace ?w2 - workspace)
    
    (arm-empty)
    (holding ?c - cube)
    (arm-folded)
    (vision-ready)

    (cube-at ?c - cube ?w - workspace)
    (free ?c - cube)

    (empty-slot ?s - slot)
    (at-slot ?c - cube ?s - slot)

    (tag-found ?t - tag ?w - workspace)
    (container-found ?k - container)
    
    (aligned-tag ?t - tag ?w - workspace)
    (container-aligned ?k - container)
  
    (container-at ?k - container ?w - workspace)
    (target-container ?c - cube ?k - container)
    (cube-in ?c - cube ?k - container)

    (precision-slot ?t - tag ?w - workspace)
    (target-slot ?c - cube ?s - slot)
    (stacking-area ?w - workspace)

    (base-free)
    (target ?c - cube ?w - workspace)

    (tag_desk ?w - workspace ?t - tag)   
    (tag_cube ?c - cube ?t - tag)        
    (cube_at_stack_base ?c - cube ?w - workspace) 
    (cube_stacked ?c_top - cube ?c_base - cube ?w - workspace) 

    ;; NOVOS PREDICADOS DE OTIMIZAÇÃO (POSITIVOS)
    (slot-enabled ?s - slot)
    (enables ?s1 - slot ?s2 - slot)
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

  (:action zero_position
    :parameters (?w - workspace)
    :precondition (and (robot-at ?w) (arm-folded) (arm-empty))
    :effect (and (not (arm-folded)) (vision-ready))
  )

  (:action reset_arm
    :parameters (?w - workspace)
    :precondition (and (robot-at ?w) (vision-ready) (arm-empty))
    :effect (and (not (vision-ready)) (arm-folded))
  )

  (:action align_tag
    :parameters (?w - workspace ?t - tag)
    :precondition (and 
      (robot-at ?w)
      (arm-empty)
      (vision-ready)
      (base-free) 
    )
    :effect (and 
      (aligned-tag ?t ?w)
      (not (base-free))
    )
  )

  (:action align-container
    :parameters (?w - workspace ?c - cube ?k - container)
    :precondition (and
      (robot-at ?w)
      (container-at ?k ?w)
      (target-container ?c ?k)
      (arm-empty)
      (vision-ready)
      (base-free)
    )
    :effect (and 
      (container-aligned ?k)
      (not (base-free))
    )
  )

  ;; GET simplificado: O robô pode tirar de qualquer slot a qualquer momento
  (:action get_cube_slot
    :parameters (?c - cube ?s - slot ?w - workspace)
    :precondition (and
      (robot-at ?w)
      (arm-empty)
      (at-slot ?c ?s)
      (arm-folded)
      (target ?c ?w) 
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

  ;; PUT otimizado: Exige que o slot esteja habilitado, e habilita o próximo
  (:action put_cube_slot
    :parameters (?c - cube ?s - slot ?w - workspace ?s_next - slot)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (empty-slot ?s)
      (slot-enabled ?s)
      (enables ?s ?s_next)
      (arm-folded)
      (base-free)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (at-slot ?c ?s)
      (not (empty-slot ?s))
      (slot-enabled ?s_next)
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
      (base-free)
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
    :parameters (?w - workspace ?c - cube ?k - container)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (container-at ?k ?w)
      (container-aligned ?k)
      (target-container ?c ?k)
      (vision-ready)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-in ?c ?k)
      (not (container-aligned ?k))
      (base-free)
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
      (not (aligned-tag ?t-hole ?w))
      (base-free)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action stack_cube_base
    :parameters (?w - workspace ?c - cube ?t_desk - tag)
    :precondition (and
      (robot-at ?w)
      (stacking-area ?w)
      (holding ?c)
      (tag_desk ?w ?t_desk)      
      (aligned-tag ?t_desk ?w)   
      (vision-ready)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (cube_at_stack_base ?c ?w) 
      (free ?c)                  
      (not (aligned-tag ?t_desk ?w))
      (base-free)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action stack_cube_top
    :parameters (?w - workspace ?c_top - cube ?c_base - cube ?t_base - tag)
    :precondition (and
      (robot-at ?w)
      (stacking-area ?w)
      (holding ?c_top)
      (cube_at_stack_base ?c_base ?w) 
      (tag_cube ?c_base ?t_base)      
      (aligned-tag ?t_base ?w)        
      (vision-ready)
    )
    :effect (and
      (not (holding ?c_top))
      (arm-empty)
      (cube-at ?c_top ?w)
      (cube_stacked ?c_top ?c_base ?w)
      (not (aligned-tag ?t_base ?w))
      (base-free)
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
      (tag_cube ?c ?t)
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
)