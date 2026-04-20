#!/bin/bash

rm -f snowDetour1.txt snowDetour2.txt

echo "Compilazione in corso..."
javac -d bin -cp "lib/enhsp-20.jar:src" src/Main.java src/planners/ENHSP.java src/models/*.java

if [ $? -eq 0 ]; then
    echo "Compilazione completata. Avvio solver per chrisgur_problem..."
    java -cp bin:lib/enhsp-20.jar Main -o Project_dh/domain/Domain_three.pddl -f Project_dh/problem/3-snowmen/rob_james_matthew_problem.pddl -planner snowman
    
else
    echo "Errore di compilazione!"
fi
