#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Sherwin
#
# Created:     18/01/2012
# Copyright:   (c) Sherwin 2012
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python
import sys
import operator

def getFile(name):
    return open(name)

def getFileAppend(name):
    return open(name, 'a')

def getFileWrite(name):
    return open(name, 'w')

# Creates a dictionary keyed by first token to the tokens after it
def createDict(fileName,labels):
    mapping = {}
    file = getFile(fileName)
    for line in file:
        vector = line.rstrip('\n').split(',')
        key = vector[0]
        if not len(vector) == 1:
            value = vector[1:]
            if mapping.has_key(key):
                mapping[key].extend(value)
            else:
                mapping[key] = value
    return mapping

def getListOfLabels(fileName):
    labelList = []
    file = getFile(fileName)
    for line in file:
        labelList.append(line.rstrip('\n'))
    return labelList

def roundDown(num):
    if num >= 1:
        return 1
    else:
        return 0

def printErr():
    print '\nMalformed argument inputs!\nUsage: GroundTruth_labelmap Prediction_labelmap labelfile binary? [overwrite_arg]\n'

# Returns (# times label in file1, # times label in file2, # times file1 and file2 labeled segment with label)
def compareTwoFiles(file1, file2, file3, binaryStr, fileWrite):

    binary = False
    if binaryStr == 'yes' or binaryStr == 'y':
        binary = True
    elif binaryStr == 'no' or binaryStr == 'n':
        binary = False
    else:
        printErr()
        exit()

    labels = getListOfLabels(file3)
    dict1 = createDict(file1,labels)
    dict2 = createDict(file2,labels)

    totalDict1 = 0
    totalDict2 = 0
    totalIntersections = 0
    for label in labels:
        dict1LabelSet = set([])
        dict2LabelSet = set([])
        if dict1.has_key(label):
            dict1LabelSet = set(dict1[label])
        if dict2.has_key(label):
            dict2LabelSet = set(dict2[label])

        dict1LabelSetSize = len(dict1LabelSet)
        dict2LabelSetSize = len(dict2LabelSet)
        intersectionSize = len(dict1LabelSet.intersection(dict2LabelSet))

        if binary:
            dict1LabelSetSize = roundDown(dict1LabelSetSize)
            dict2LabelSetSize = roundDown(dict2LabelSetSize)
            intersectionSize = roundDown(intersectionSize)

        totalDict1 = totalDict1 + dict1LabelSetSize
        totalDict2 = totalDict2 + dict2LabelSetSize
        totalIntersections = totalIntersections + intersectionSize

        strLst = [label,',',str(dict1LabelSetSize),',',str(dict2LabelSetSize),',',str(intersectionSize),'\n']
        fileWrite.write(''.join(strLst))

    return totalDict1, totalDict2, totalIntersections

def precision(TPITriplet):
    truth = TPITriplet[0]
    if truth == 0:
        return 0
    intersection = TPITriplet[2]
    return float(intersection)/truth

def recall(TPITriplet):
    prediction = TPITriplet[1]
    if prediction == 0:
        return 0
    intersection = TPITriplet[2]
    return float(intersection)/prediction

def printAll():
    printTPI(TPI)
    printPrecision(TPI)
    printRecall(TPI)

def printTPI(TPI):
    print '============================================'
    print 'Predictions: ', TPI[1]
    print 'Truths: ', TPI[0]
    print 'Correct Predictions: ', TPI[2]

def printPrecision(TPITriplet):
    print '============================================'
    print 'Precision:'
    print '\t',precision(TPITriplet)

def printRecall(TPITriplet):
    print '============================================'
    print 'Recall:'
    print '\t',recall(TPITriplet)
    print '============================================'

def main():
    # TPI = Truth/Precision/Intersection triplet
    fileWrite = getFileAppend('out')
    if len(sys.argv) == 6:
        fileWrite = getFileWrite('out')
    elif not len(sys.argv) == 5:
        printErr()
        exit()

    TPI = compareTwoFiles(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], fileWrite)

    print '\nSuccessful Evaluation'
    printTPI(TPI)
    printPrecision(TPI)
    printRecall(TPI)
    pass

if __name__ == '__main__':
    main()
