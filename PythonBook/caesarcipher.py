SYMBOLS = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'
MAX_KEY_SIZE=len(SYMBOLS) #shows that the key in the cipher should be btwn. 1 and 52, cuz 26+26=52

def getMode():
    while True:
        print('Do you wish to encrypt or decrypt a message?')
        mode=input().lower() #returns auto lowercase response from the user
        if mode in ['encrypt', 'e', 'decrypt', 'd']:
            return mode
        else:
            print('Enter either "encrypt" or "e" or "decrypt" or "d".') #if user does not type any of the four responses, then the function will ask the user to type again.

def getMessage():
    print('Enter your message:')
    return input()

def getKey():
    key = 0
    while True: #makes sure the function keeps looping until the user enters a valid key btwn. 1 and 52
        print('Enter the key number (1-%s)' % (MAX_KEY_SIZE))
        key = int(input()) #seetes key to integer 
        if (key >=1 and key <= MAX_KEY_SIZE):
            return key
        
def getTranslatedMessage(mode, message, key): #message = text to be ciphered or deciphered
    if mode[0] == 'd': #mode = encrypt or decrypt
        key = -key #key = key used in cipher
    translated = ''

    for symbol in message:
        symbolIndex = SYMBOLS.find(symbol)
        if symbolIndex == -1:
            translated += symbol
        else:
            symbolIndex += key
            if symbolIndex >= len(SYMBOLS):
                symbolIndex -= len(SYMBOLS)
            elif symbolIndex < 0:
                symbolIndex += len(SYMBOLS)

            translated += SYMBOLS[symbolIndex]
    return translated

mode = getMode()
message = getMessage()
key= getKey()
print('Your translated text is:')
print(getTranslatedMessage(mode, message, key))
