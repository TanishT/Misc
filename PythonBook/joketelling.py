import random
import sys
import math

def getNewBoard():
  board = []
  for x in range(60):
    board.append([])
    for y in range(15):
      if random.randint(0, 1) == 0:
        board[x].append('~')
      else:
        board[x].append('`')
  return board

def drawBoard(board):
  tensDightsLine = '  '
  for i in range(1,6):
    tensDightsLine += (' ' * 9) + str(i)

  print(tensDightsLine)
  print('  ' + ('0123456789' * 6))
  print()

  for row in range(15):
    boardRow = ''
    for column in range(60):
      boardRow += board[column][row]

    print('%2d %s %s' % (row, boardRow, row))

  print()
  print(' ' + ('0123456789' * 6))
  print(tensDightsLine)


# run code below
b=getNewBoard()
drawBoard(b)

def getRandomChests(numChests):
  chests = []
  while len(chests) < numChests :
    newChest = [random.randint(0,59), random.randint(0,14)]
    if newChest not in chests :
      chests.append(newChest)
  return chests

b=getRandomChests(5)
print(b)

def isOnBoard(x,y):
  return x>=0 and x <=59 and y >= 0 and y<=14

def makeMove(board, chests, x, y):
  smallestDistance = 100
  for cx, cy in chests:
    distance = math.sqrt((cx-x) * (cx-x) + (cy-y) * (cy-y))

    if distance < smallestDistance:
      smallestDistance = distance

  smallestDistance = round(smallestDistance)

  if smallestDistance == 0:
    chests.remove([x,y])
    return 'Good Work, you have found a sunken treasure chest!'
  else:
    if smallestDistance <10:
      board[x][y] = str(smallestDistance)
      return 'Treasure chest detected at a distance of %s from the sonar device.' % (smallestDistance)
    else:
      board[x][y]: 'X'
      return 'Unfortunately, Sonar Device did not detect anything. Try again.'

def enterPlayerMove(previousMoves):
  print('Where do you want to drop your sonar device? (0-59) (0-14)')
  while True:
    move = input()
    if move.lower == 'quit':
      print('Thanks for playing!')
      sys.exit()

    move = move.split()
    if len(move) == 2 and move[0].isdigit() and move[1].isdigit() and isOnBoard(int(move[0]), int(move[1])):
       if[int(move[0]), int(move[1])] in previousMoves :
         print('You have already moved there. Pick another spot')
         continue
       else:
         return [int(move[0]), int(move[1])]
      
    print('Enter a number from 0 - 59, a space, and then a number from 0 - 14.')

def showInstructions():
  print(''' Instructions: Senor, you are the unfortunate captain of the JeremyZhou, a treasure hunting ship. Your current mission, should you choose to accept it, is to use sonar devices to find sunken treasure chests at the bottom of the ocean.
  Press enter to continue...''')
  input()
  print('''However, you only have the cheap sonar panels that finds the distance away from the treasure chests, not direction. Enter the coordinates to drop a sonar device. Press enter to continue...''')
  input()
  print('''The ocean map will be marked with how far away the nearest chest is, and an X will be placed if it the treasure chest is out of range from your sonar device. Press enter to continue... ''')
  input()
  print('''When you place a sonar device directly on a chest, you retrieve it. The treasure chests don't move around. Sonar Devices can detect treasure chests and a maximum range of 9 spaces. Try to collect all the treaure chests. Good luck! Press enter to continue...''')
  input()

print('Would you like view the instructions? (yes/no)')
answer=input()
if answer.lower().startswith('y'):
  showInstructions()

while True:
  sonarDevices = 20
  theBoard = getNewBoard()
  theChests = getRandomChests(3)
  drawBoard(theBoard)
  previousMoves = []

  while sonarDevices > 0:
    print('You have %s sonar device(s) left. %s treasure chest(s) remaining.' % (sonarDevices, len(theChests)))
    x,y = enterPlayerMove(previousMoves)
    previousMoves.append([x,y])

    moveResult = makeMove(theBoard, theChests, x,y)
    drawBoard(theBoard)
    if moveResult == False:
      continue
    else:
      if moveResult == 'You have found a sunken treasure chest!':
        for x,y in previousMoves:
          makeMove(theBoard, theChests, x, y)
          drawBoard(theBoard)
          print(moveResult)
    if len(theChests) == 0:
      print('You have found all the sunken treasure chests! GG')
      break

    sonarDevices -= 1 

  if sonarDevices == 0:
      print('We\'ve run out of sonar devices! Unfortunate! Now we have to retreat and head home. Game over.')
      print('   The remaining chests were here:')
      for x, y in theChests:
        print('   %s, %s,' % (x,y))

  print('Do you want to play again? (yes/no)')
  answer2=input()
  if answer2.lower().startswith('n'):
    sys.exit()
    
