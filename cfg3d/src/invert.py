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
    print '\nUsage: fileToBeInverted labelToLabelIndexMapping'

def createMappingDict(mappingFileName):
    mapping = {}
    file = getFile(mappingFileName)
    for line in file:
        vector = line.rstrip('\n').split(',')
        key = vector[0]
        value = vector[1]
        if mapping.has_key(key):
            sys.exit("Duplicate labeling")
        else:
            mapping[key] = value
    return mapping

def invert(fileName, mappingFileName):
    mapping = {}
    file = getFile(fileName)
    writeFile = getFileWrite(''.join([fileName,'_inverted']))

    mappingDict = createMappingDict(mappingFileName)

    for line in file:
        vector = line.rstrip('\n').split(',')
        head = vector[0]
        if not len(vector) == 1:
            tail = vector[1:]
            for elem in tail:
                if mappingDict.has_key(head):
                    writeFile.write(''.join([elem,' ',mappingDict[head],'\n']))
                else:
                    sys.exit(''.join(['No corresponding labelIndex in labelToLabelIndexFile: ',head]))

def main():
    if len(sys.argv) == 3:
        invert(sys.argv[1], sys.argv[2])
    else:
        printUsage()
    pass

if __name__ == '__main__':
    main()
