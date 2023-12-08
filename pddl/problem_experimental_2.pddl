(define (problem problem_experimetal_2) (:domain experimental_2)
(:objects 
    wp0 - home
    wp1 wp2 wp3 wp4 - waypoint
    mk11 mk12 mk13 mk15 - marker
)

(:init
    (robot_at wp0)
    (visible mk11 wp1) (visible mk12 wp2) (visible mk13 wp3) (visible mk15 wp4)
    
    )

(:goal (and
    (seen mk11) (seen mk12) (seen mk13) (seen mk15)
    (robot_at wp0)
))

)

