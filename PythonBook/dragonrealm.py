import random
import time

def displayIntro():
    print('You are in a land full of dragons. In front of you, lie two caves. In one of the caves, the dragon is friendly and will share his treasure with you. In the other cave, the dragon is greedy and hungry, and will yoink you on sight.')
    print()
    #intro instructions for the player

def chooseCave():
    cave = ''
    while cave != '1' and cave != '2':
        print('Which cave will you go into? (1 or 2)')
        cave = input()

    return cave

#asks the player which cave they will choose: 1 or 2

def checkCave(chosenCave):
    print('You approach the cave...')
    time.sleep(2) #2 second break btwn. each statement
    print('It is dark and spooky...')
    time.sleep(2)
    print('A large dragon jumps out in front of you! He opens his jaws and..')
    print()
    time.sleep(2)

    friendlyCave = random.randint(1,2) #asks computer to choose a random number 1 or 2 and assign that number to the variable friendly cave/

    if chosenCave == str(friendlyCave):
        print('Gives you his treasure!') #if number player chooses is equal to friendly cave.
    else:
        print('Yoinks you into his mouth in one bite!') #if number player chooses is not equal to friendly cave.

playAgain = 'yes'
while playAgain == 'yes' or playAgain == 'y':
    displayIntro()
    caveNumber = chooseCave()
    checkCave(caveNumber)

    print('Do you want to play again? (yes or no)')
    playAgain = input()
             
#if player wants to play again
