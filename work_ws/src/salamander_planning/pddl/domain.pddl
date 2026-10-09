(define (domain salamander-domain)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    workspace
    cube
    tag
    slot
    arm-position
    height
  )

  (:constants
    initial-position
    table-position
    shelf-position
    precision-position
    stacking-position
    container-position
    - arm-position
  )

  (:predicates
    (robot-at ?w - workspace)
    (connected ?w1 - workspace ?w2 - workspace)

    (arm-at ?p - arm-position)
    (arm-empty)
    (holding ?c - cube)

    (cube-at ?c - cube ?w - workspace)
    (cube-tag ?c - cube ?t - tag)
    (free ?c - cube)

    (empty-slot ?s - slot)
    (at-slot ?c - cube ?s - slot)
    (slot-position ?s - slot ?p - arm-position)

    (tag-found ?t - tag ?w - workspace)
    (container-found ?w - workspace)
    
    (aligned-tag ?t - tag ?w - workspace)
    (container-aligned ?w - workspace)

    (precision-slot ?t - tag ?w - workspace)
    (stacking-area ?w - workspace)
    (on-top ?c-top - cube ?c-bottom - cube)
    (cube-height ?c - cube ?h - height)
    (next-height ?h1 - height ?h2 - height)
  )

  (:action move
    :parameters (?origin - workspace ?destination - workspace)
    :precondition (and
      (robot-at ?origin)
      (connected ?origin ?destination)
      (arm-at initial-position)
    )
    :effect (and
      (not (robot-at ?origin))
      (robot-at ?destination)
    )
  )

  ; (:action find-cube
  ;  :parameters (?c - cube ?t - tag ?w - workspace)
  ;  :precondition (and (robot-at ?w) (cube-at ?c ?w) (cube-tag ?c ?t))
  ;  :effect (and 
  ;    (tag-found ?t ?w)
  ;  )
  ; )

  (:action find-container
    :parameters (?w - workspace)
    :precondition (robot-at ?w)
    :effect (and 
      (container-found ?w)
    )
  )

  (:action align_to_cube
    :parameters (?w - workspace ?c - cube ?t - tag)
    :precondition (and 
      (robot-at ?w) 
      (cube-at ?c ?w) 
      (cube-tag ?c ?t)
    )
    :effect (and 
      (aligned-tag ?t ?w)
    )
  )

  (:action align-tag
    :parameters (?w - workspace ?t - tag)
    :precondition (and (robot-at ?w) (arm-empty) (tag-found ?t ?w))
    :effect (and 
      (aligned-tag ?t ?w)
    )
  )

  (:action align-container
    :parameters (?w - workspace)
    :precondition (and (robot-at ?w) (container-found ?w))
    :effect (and 
      (container-aligned ?w)
    )
  )


  (:action get_cube_slot
    :parameters (?c - cube ?s - slot ?pos-s - arm-position)
    :precondition (and
      (arm-empty)
      (at-slot ?c ?s)
      (slot-position ?s ?pos-s)
      (arm-at initial-position)
    )
    :effect (and
      (not (arm-empty))
      (holding ?c)
      (not (at-slot ?c ?s))
      (empty-slot ?s)
      (not (arm-at initial-position))
      (arm-at ?pos-s)
    )
  )

  (:action put_cube_table
    :parameters (?w - workspace ?c - cube)
    :precondition (and (robot-at ?w) (holding ?c) (arm-at initial-position))
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (free ?c)
      (not (arm-at initial-position))
      (arm-at table-position)
    )
  )

  (:action put-cube-container
    :parameters (?w - workspace ?c - cube)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (container-aligned ?w)
      (arm-at initial-position)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (not (arm-at initial-position))
      (arm-at container-position)
    )
  )

  (:action put-cube-shelf
    :parameters (?w - workspace ?c - cube)
    :precondition (and (robot-at ?w) (holding ?c) (arm-at initial-position))
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (not (arm-at initial-position))
      (arm-at shelf-position)
    )
  )

  (:action put-cube-precision
    :parameters (?w - workspace ?c - cube ?t-hole - tag)
    :precondition (and
      (robot-at ?w)
      (holding ?c)
      (aligned-tag ?t-hole ?w)
      (precision-slot ?t-hole ?w)
      (arm-at initial-position)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (cube-at ?c ?w)
      (not (arm-at initial-position))
      (arm-at precision-position)
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
      (arm-at initial-position)
    )
    :effect (and
      (not (holding ?c-held))
      (arm-empty)
      (cube-at ?c-held ?w)
      (on-top ?c-held ?c-base)
      (not (free ?c-base))
      (free ?c-held)
      (cube-height ?c-held ?alt-new)
      (not (arm-at initial-position))
      (arm-at stacking-position)
    )
  )

  (:action get_cube_table
    :parameters (?w - workspace ?c - cube ?t - tag)
    :precondition (and
      (robot-at ?w)
      (arm-empty)
      (aligned-tag ?t ?w)   ;; O robô PRECISA estar alinhado com a tag
      (cube-tag ?c ?t)     ;; Essa tag precisa pertencer a esse cubo
      (cube-at ?c ?w)
      (free ?c)
      (arm-at initial-position)
    )
    :effect (and
      (not (arm-empty))
      (holding ?c)
      (not (cube-at ?c ?w))
    )
  )

  (:action put_cube_slot
    :parameters (?c - cube ?s - slot ?pos-s - arm-position)
    :precondition (and
      (holding ?c)
      (empty-slot ?s)
      (slot-position ?s ?pos-s)
      (arm-at initial-position)
    )
    :effect (and
      (not (holding ?c))
      (arm-empty)
      (at-slot ?c ?s)
      (not (empty-slot ?s))
    )
  )
)