(define (domain salamander-domain)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    workspace
    cube
    tag
    slot
    table-slot
    container
    color
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

    (tslot-at ?l - table-slot ?w - workspace)
    (table-slot-free ?l - table-slot)
    (cube-in-tslot ?c - cube ?l - table-slot)
    (target-slot ?c - cube ?s - slot)
    (aligned-precision ?t - tag ?w - workspace)
    (precision-started)
    (stacking-area ?w - workspace)

    (base-free)
    (target ?c - cube ?w - workspace)

    (tag-desk ?w - workspace ?t - tag)   
    (tag-cube ?c - cube ?t - tag)        
    (cube-stacked-top ?c_top - cube ?c_base - cube ?w - workspace) 
    (cube-stacked-base ?c_top - cube ?c_base - cube ?w - workspace) 

    (shelf ?w - workspace)    
    (on-top-shelf ?c - cube ?w - workspace)

    (cube-color ?c - cube ?col - color)
    (precision-area ?w - workspace)
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

  (:action align_container
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
      (vision-ready)
    )
  )

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

  (:action put_cube_slot
    :parameters (?c - cube ?s - slot ?w - workspace)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (empty-slot ?s)
      (arm-folded)
      (base-free)
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

  (:action put_cube_table
    :parameters (?w - workspace ?c - cube)
    :precondition (and (robot-at ?w) (holding ?c) (vision-ready)(base-free))
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (free ?c)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action put_cube_shelf
    :parameters (?w - workspace ?c - cube)
    :precondition (and (robot-at ?w) (shelf ?w) (holding ?c) (base-free))
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (on-top-shelf ?c ?w)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action put_cube_container
    :parameters (?w - workspace ?c - cube ?k - container ?s - slot)
    :precondition (and
      (robot-at ?w)
      (arm-empty)
      (at-slot ?c ?s)
      (container-at ?k ?w)
      (container-aligned ?k)
      (target-container ?c ?k)
      (vision-ready)
    )
    :effect (and
      (not (at-slot ?c ?s))
      (empty-slot ?s)
      (cube-in ?c ?k)
      (not (container-aligned ?k))
      (base-free)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action put-cube-precision
    :parameters (?w - workspace ?c - cube ?t - tag ?l - table-slot)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (tslot-at ?l ?w)
      (table-slot-free ?l)
      (vision-ready))
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (cube-in-tslot ?c ?l)
      (not (table-slot-free ?l))
      (not (aligned-tag ?t ?w))
      (base-free)
      (not (vision-ready))
      (arm-folded)
    )
  )


  (:action stack_cube_base
    :parameters (?w - workspace ?c_base - cube ?s1 - slot ?t_desk - tag ?c_top - cube)
    :precondition (and
      (robot-at ?w)
      (arm-empty)
      (stacking-area ?w)
      (tag-desk ?w ?t_desk)      
      (aligned-tag ?t_desk ?w)
      (at-slot ?c_base ?s1)
      (vision-ready)
    )
    :effect (and
      (cube-at ?c_base ?w)
      (arm-empty)
      (cube-stacked-base ?c_top ?c_base ?w) 
      (free ?c_base)                  
      (not (aligned-tag ?t_desk ?w))
      (base-free)
      (not (vision-ready))
      (arm-folded)
      (empty-slot ?s1)
      (not (at-slot ?c_base ?s1))
    )
  )

  (:action stack_cube_top
    :parameters (?w - workspace ?c_top - cube ?s2 - slot ?c_base - cube ?t_base - tag)
    :precondition (and
      (robot-at ?w)
      (stacking-area ?w)
      (at-slot ?c_top ?s2)
      (cube-stacked-base ?c_top ?c_base ?w)
      (tag-cube ?c_base ?t_base)      
      (aligned-tag ?t_base ?w)        
      (vision-ready)
    )
    :effect (and
      (cube-at ?c_top ?w)
      (cube-stacked-top ?c_top ?c_base ?w)
      (not (aligned-tag ?t_base ?w))
      (base-free)
      (not (vision-ready))
      (arm-folded)
      (empty-slot ?s2)
      (not (at-slot ?c_top ?s2))
    )
  )

  (:action get-cube-precision
    :parameters (?w - workspace ?c - cube ?t - tag ?l - table-slot)
    :precondition (and
      (robot-at ?w)
      (arm-empty)
      (aligned-precision ?t ?w)
      (tag-cube ?c ?t)
      (cube-in-tslot ?c ?l)
      (vision-ready))
    :effect (and
      (not (arm-empty))
      (holding ?c)
      (not (cube-at ?c ?w))
      (not (cube-in-tslot ?c ?l))
      (table-slot-free ?l)
      (not (aligned-precision ?t ?w))
      (base-free)
      (not (vision-ready))
      (arm-folded)
    )
  )

  (:action align_tag_precision
    :parameters (?w - workspace)
    :precondition (and
      (robot-at ?w)
      (precision-area ?w)
      (arm-empty)
      (vision-ready)
      (base-free))
    :effect (and
      (precision-started)
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
      (tag-cube ?c ?t)
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