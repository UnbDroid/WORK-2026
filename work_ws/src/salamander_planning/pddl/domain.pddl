(define (domain exercise0)
  (:requirements :strips :typing :negative-preconditions :disjunctive-preconditions :equality)

  (:types robot location object)

  (:predicates
    (at-robot ?r - robot ?loc - location)
    (at-object ?o - object ?loc - location)
    (arm-empty)
    (holding ?r - robot ?o - object)
    (visited ?loc - location) ;para ele ir para outro wp de forma autonoma
    (connected ?from - location ?to - location) ;seguir as rotas
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at-robot ?r ?from) (connected ?from ?to))
    :effect (and (not (at-robot ?r ?from)) (at-robot ?r ?to) (visited ?to))
  )    

  (:action pick_up
    :parameters (?r - robot ?o - object ?loc - location)
    :precondition (and (at-robot ?r ?loc) (at-object ?o ?loc) (arm-empty))
    :effect (and (not (at-object ?o ?loc)) (holding ?r ?o) (not (arm-empty)))
  )

  (:action put_down
    :parameters (?r - robot ?o - object ?loc - location)
    :precondition (and (holding ?r ?o) (at-robot ?r ?loc) (not (arm-empty)))
    :effect (and (not (holding ?r ?o)) (at-object ?o ?loc) (arm-empty))
  )
)