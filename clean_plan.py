import os
import sys

def clean_plan():
    path_1 = 'Project_dh/plan/1-snowman'
    path_2 = 'Project_dh/plan/2-snowmen'
    path_3 = 'Project_dh/plan/3-snowmen'
    
    for path in [path_1,
    path_2,
    path_3
    ]:
        os.system(f'rm {path}/*.txt')
    
    print("Piani puliti con successo.")

if __name__ == "__main__":
    clean_plan()