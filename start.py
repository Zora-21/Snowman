import os
def start():
    os.system("python3 /Users/matteo/Snowman/clean_plan.py")
    os.system('javac -d bin -cp "lib/enhsp-20.jar:src" src/Main.java src/planners/ENHSP.java src/models/*.java')
    os.system("java -cp bin:lib/enhsp-20.jar Main")

if __name__ == "__main__":
    start()
