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

# Returns (# times label in file1, # times label in file2, # times file1 and file2 labeled segment with label)
def compareTwoFiles(file1, file2, file3, fileWrite):
    labels = getListOfLabels(file3)
    dict1 = createDict(file1,labels)
    print dict1, '\n'
    dict2 = createDict(file2,labels)
    print dict2, '\n'

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
        totalDict1 = totalDict1 + len(dict1LabelSet)
        totalDict2 = totalDict2 + len(dict2LabelSet)
        totalIntersections = totalIntersections + len(dict1LabelSet.intersection(dict2LabelSet))

        strLst = [label,',',str(len(dict1LabelSet)),',',str(len(dict2LabelSet)),',',str(len(dict1LabelSet.intersection(dict2LabelSet))),'\n']
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
    if len(sys.argv) == 5:
        fileWrite = getFileWrite('out')
    elif not len(sys.argv) == 4:
        print '\nMalformed argument inputs!\nUsage: GroundTruth_labelmap Prediction_labelmap labelfile [overwrite_arg]\n'
        exit()

    TPI = compareTwoFiles(sys.argv[1], sys.argv[2], sys.argv[3], fileWrite)

##    strLst = [str(TPI[0]),',',str(TPI[1]),',',str(TPI[2]),'\n']
##    fileWrite.write(''.join(strLst))
    pass

if __name__ == '__main__':
    main()
