;Header and description

(define (domain experimental_2)

;remove requirements that are not needed
(:requirements 
	:strips 
	:fluents 
	:typing 
	:disjunctive-preconditions 
	:durative-actions)

(:types 
    waypoint
    marker
    home
)

(:predicates 
    (robot_at ?x - (either home waypoint))
    (seen ?mk - marker)
    (visible ?mk - marker ?wp - waypoint)
)

;define actions here
(:durative-action move
    :parameters (?from ?to - waypoint)
    :duration( = ?duration 30)
    :condition (at start (robot_at ?from))
    :effect (and (at end (robot_at ?to)) (at start (not(robot_at ?from))))
)

(:durative-action go_home
    :parameters (?from - waypoint ?to - home)
    :duration( = ?duration 30)
    :condition (at start (robot_at ?from))
    :effect (and (at end (robot_at ?to)) (at start (not(robot_at ?from))))
)

(:durative-action leave_home
    :parameters (?from - home ?to - waypoint)
    :duration( = ?duration 30)
    :condition (at start (robot_at ?from))
    :effect (and (at end (robot_at ?to)) (at start (not(robot_at ?from))))
)

(:durative-action detect
    :parameters (?wp - waypoint ?mk - marker)
    :duration( = ?duration 10)
    :condition (and (over all (robot_at ?wp)) (over all (visible ?mk ?wp)))
    :effect (at end (seen ?mk))
)

)
