(define (domain one_snowman)
    (:requirements
        :typing                   
        :negative-preconditions   
        :equality               
        :disjunctive-preconditions 
        :conditional-effects       
        :action-costs          
        :fluents                  
        :numeric-fluents       
    )

    (:types
        location direction ball - object 
    )

    (:predicates
        (snow ?l - location)          
        (next ?from ?to - location ?dir - direction) 
        (occupancy ?l - location)      
        (character_at ?l - location)  
        (ball_at ?b - ball ?l - location)                         
   )

    (:functions
        (total-cost) - number     
        (ball_size ?b - ball) - number 
        (goal) - number
    )

;--------------------------------------------------------------------------------------------------------------------------------------------------------------------
; ACTION MOVE CHARACTER
;--------------------------------------------------------------------------------------------------------------------------------------------------------------------
    (:action move_character
     :parameters
       (?from ?to - location ?dir - direction) 
     :precondition
        (and
            (next ?from ?to ?dir)    
            (character_at ?from)     
            (not (occupancy ?to)))   
     :effect
        (and
            (not (character_at ?from))
            (character_at ?to))       
    )

;--------------------------------------------------------------------------------------------------------------------------------------------------------------------
; ACTION MOVE BALL
;--------------------------------------------------------------------------------------------------------------------------------------------------------------------
    (:action move_ball
     :parameters
        (?b - ball ?ppos ?from ?to - location ?dir - direction)
     :precondition
        (and
            (next ?ppos ?from ?dir)
            (next ?from ?to ?dir)
            (ball_at ?b ?from)
            (character_at ?ppos)
            
;---------- REGOLA DI IMPILAMENTO: Nella posizione di partenza, la palla da spostare può essere impilata solo su palle più grandi -----------------------------------
            (forall (?o - ball)
                (or
                    (= ?o ?b)          
                    (not (ball_at ?o ?from)) 
                    (< (ball_size ?b) (ball_size ?o))    
                )
            ) 
            
;---------- REGOLA DI MOVIMENTO: O la palla è sola nella posizione di partenza, oppure la posizione di arrivo deve essere libera -------------------------------------
            (or
                (forall (?o - ball)
                    (or
                        (= ?o ?b)
                        (not (ball_at ?o ?from)))) 
                (forall (?o - ball)
                        (not (ball_at ?o ?to))        
                )
            )    
            
;---------- REGOLA DI IMPILAMENTO 2: Nella posizione di arrivo, la palla può essere impilata solo su palle più grandi -----------------------------------------------
            (forall (?o - ball)
                    (or
                        (not (ball_at ?o ?to))   
                        (< (ball_size ?b) (ball_size ?o))    
                    )        
            ) 
        )

     :effect
        (and
            (occupancy ?to)
            (not (ball_at ?b ?from))
            (ball_at ?b ?to)
            
            (when
                (forall (?o - ball)
                    (or
                        (= ?o ?b)
                        (not (ball_at ?o ?from))))
                (and
                    (not (character_at ?ppos))  
                    (character_at ?from)        
                    (not (occupancy ?from)))    
            )  
            
;---------- Se la palla è spostata su una cella innevata, la palla spostata aumenta di dimensione -------------------------------------------------------------------
            (when
                (and
                    (snow ?to)
                    (< (ball_size ?b) 3)) 
                (increase (ball_size ?b) 1))
            
            (not (snow ?to))
            (increase (total-cost) 1)  
        )
    )

;--------------------------------------------------------------------------------------------------------------------------------------------------------------------
; ACTION GOAL
;--------------------------------------------------------------------------------------------------------------------------------------------------------------------    
    (:action goal
    :parameters
        (?b_p ?b_m ?b_g - ball ?l - location)
    :precondition
        (and
            (not (= ?b_p ?b_m))
            (not (= ?b_p ?b_g))
            (not (= ?b_m ?b_g))

            (ball_at ?b_p ?l)
            (ball_at ?b_m ?l)
            (ball_at ?b_g ?l)

            (= (ball_size ?b_p) 1)  
            (= (ball_size ?b_m) 2)  
            (= (ball_size ?b_g) 3) 
        )
    :effect
        (assign(goal) 1)
    )
)