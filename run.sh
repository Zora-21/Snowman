#!/bin/bash

echo "Compilazione in corso..."
javac -d bin -cp "lib/enhsp-20.jar:src" src/Main.java src/planners/ENHSP.java src/models/*.java

if [ $? -eq 0 ]; then
    echo "Compilazione completata. Avvio solver per chrisgur_problem..."
    java -cp bin:lib/enhsp-20.jar Main -o Project_dh/domain/Domain_two.pddl -f Project_dh/problem/2-snowmen/unnamed_problem.pddl -planner snowman
    
else
    echo "Errore di compilazione!"
fi
