#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Sherwin
#
# Created:     21/01/2012
# Copyright:   (c) Sherwin 2012
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python
import sys

def getFile(name):
    return open(name)

def getFileWrite(name):
    return open(name, 'w')

def printUsage():
    print '\nUsage: fileToBeInverted'

def invert(fileName):
    mapping = {}
    file = getFile(fileName)
    writeFile = getFileWrite(''.join([fileName,'_inverted']))
    for line in file:
        vector = line.rstrip('\n').split(',')
        head = vector[0]
        if not len(vector) == 1:
            tail = vector[1:]
            for elem in tail:
                writeFile.write(''.join([head,',',elem,'\n']))

def main():
    if len(sys.argv) == 2:
        invert(sys.argv[1])
    else:
        printUsage()
    pass

if __name__ == '__main__':
    main()
