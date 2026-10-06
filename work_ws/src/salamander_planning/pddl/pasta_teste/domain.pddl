(define (domain salamander-domain)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    workspace
    cube
    tag
    ; slot
    ; arm-position
    ; height
  )

  ; (:constants
  ;   initial-position table-position shelf-position precision-position stacking-position container-position - arm-position
  ; )

  (:predicates
    (robot-at ?w - workspace)
    (connected ?w1 - workspace ?w2 - workspace)

    (cube-at ?c - cube ?w - workspace)
    (cube-tag ?c - cube ?t - tag)
    (aligned-tag ?t - tag ?w - workspace)

    ; --- PREDICADOS FUTUROS COMENTADOS ---
    ; (arm-at ?p - arm-position)
    ; (arm-empty)
    ; (holding ?c - cube)
    ; (free ?c - cube)
    ; (empty-slot ?s - slot)
    ; (at-slot ?c - cube ?s - slot)
    ; (slot-position ?s - slot ?p - arm-position)
    ; (tag-found ?t - tag ?w - workspace)
    ; (container-found ?w - workspace)
    ; (container-aligned ?w - workspace)
    ; (precision-slot ?t - tag ?w - workspace)
    ; (stacking-area ?w - workspace)
    ; (on-top ?c-top - cube ?c-bottom - cube)
    ; (cube-height ?c - cube ?h - height)
    ; (next-height ?h1 - height ?h2 - height)
  )

  ; (:functions (total-cost) - number)

  (:action move
    :parameters (?origin - workspace ?destination - workspace)
    :precondition (and
      (robot-at ?origin)
      (connected ?origin ?destination)
    )
    :effect (and
      (not (robot-at ?origin))
      (robot-at ?destination)
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

  ; --- AÇÕES FUTURAS COMENTADAS) ---
  ; (:action find-cube ...)
  ; (:action find-container ...)
  ; (:action align-container ...)
  ; (:action reset-arm ...)
  ; (:action get-cube-table ...)
  ; (:action get-cube-slot ...)
  ; (:action put-cube-table ...)
  ; (:action put-cube-container ...)
  ; (:action put-cube-shelf ...)
  ; (:action put-cube-precision ...)
  ; (:action put-cube-pile ...)
  ; (:action put-cube-slot ...)
