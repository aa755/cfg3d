'''
Created on Feb 7, 2012

@author: lisherwin
'''
import sys

def getFile(name):
    return open(name)
def getFileAppend(name):
    return open(name, 'a')
def getFileWrite(name):
    return open(name, 'w')

def parseLabelLine(line):
    vect = line.split(',')
    seg = None
    label = None
    try:
        label = vect[0]
        seg = vect[1:]
        seg[-1] = seg[-1].rstrip('\n')
        seg[0] = seg[0].lstrip(' ')
    except:
        print 'Unable to token split'
        raise
    return seg, label

def parseNeighborMapLine(line):
    vect = line.split(',')
    s = set()
    for seg in vect:
        s.add(seg.rstrip('\n'))
    return s
    
def handleInputs(labelMap, neighborMap, outputFileName):
    labelMapFile = getFile(labelMap)
    neighborMapFile = getFile(neighborMap)
    outputTreeFile = getFileWrite(outputFileName)
    observedSegsInLabelMap = set([])
    
    outputTreeFile.write('digraph {\n')
    
    for line in labelMapFile:
        segs, label = parseLabelLine(line)
        for seg in segs:
            observedSegsInLabelMap.add(seg)
            outputTreeFile.write(''.join(['\tTerminal__', seg, '__', label, ' ;\n']))
        
    segmentSet = set([])
    for line in neighborMapFile:
        additionalSegmentSet = parseNeighborMapLine(line)
        segmentSet = segmentSet.union(additionalSegmentSet)
    differenceSet = segmentSet.difference(observedSegsInLabelMap)
        
    for seg in differenceSet:
        outputTreeFile.write(''.join(['\tTerminal__', seg, ' ;\n']))
        
    outputTreeFile.write('}')
      
def printErr():
    print '\tUsage: toInitialParseTree <labelMap> <neighborMap> <outputFileName>'
    
if __name__ == '__main__':
    if len(sys.argv) == 4:
        handleInputs(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        printErr()