'''
Created on Feb 26, 2012

@author: lisherwin
'''

import pprint
import sys

def getFile(name):
    return open(name)
def getFileAppend(name):
    return open(name, 'a')
def getFileWrite(name):
    return open(name, 'w')

def handleLine(line, out):
    if not '}' in line:
        out.write(line)

def handleTreeFile(treeFile, out):
    for line in treeFile:
        handleLine(line, out)

def handleNeighborFile(neighborFile):
    segs = set()
    for line in neighborFile:
        vect = line.split(',')
        for elem in vect:
            segs.add(elem.rstrip('\n'))
    return segs

def getTreeSegs(treeFileName):
    treeFile = getFile(treeFileName)
    treeSegs = set()
    for line in treeFile:
        print line
        vect = line.split('__')
        if 'Terminal' in vect and not '->' in vect:
            treeSegs.add(vect[1].rstrip('\n').rstrip(' ;'))
        elif 'Terminal' in vect and '->' in vect:
            treeSegs.add(vect[2].rstrip('\n').rstrip(' ;'))
    return treeSegs

def handleFiles(treeFileName, neighborFileName):
    neighborFile = getFile(neighborFileName)
    out = getFileWrite(''.join([treeFileName,'.complete']))

    treeFile = getFile(treeFileName)
    handleTreeFile(treeFile, out)
    allSegs = handleNeighborFile(neighborFile)
    treeSegs = getTreeSegs(treeFileName)
    
    pprint.pprint(treeSegs)
    
    unincludedSegs = allSegs.difference(treeSegs)
    for seg in unincludedSegs:
        out.write(''.join(['Terminal__',seg,' ;\n']))
    out.write('}')

if __name__ == '__main__':
    inputFile = sys.argv[1]
    neighborFile = sys.argv[2]
    handleFiles(inputFile, neighborFile)
    
    
    