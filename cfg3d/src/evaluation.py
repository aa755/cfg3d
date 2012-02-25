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
import pprint

def getFile(name):
    return open(name)

def getFileAppend(name):
    return open(name, 'a')

def getFileWrite(name):
    return open(name, 'w')

def print2DDict(d, file, labels):
##    for l in d.keys():
##        file.write(''.join([l,'\t']))
##
##    file.write('\n')
    for outerLabel in labels:
        for innerLabel in labels:
            if d.has_key(outerLabel): 
                if d[outerLabel].has_key(innerLabel):
                    file.write(''.join([str(d[outerLabel][innerLabel]),',']))
    file.write('\n')
    file.write(','.join(labels))
#    for row in d.keys():
#        length = len(row)
#        spaces = 16 - length
#        front = row
#        spaceList = [' ']*spaces
#        frontList = [front]
#        frontList.extend(spaceList)
#        file.write(''.join(frontList))
#
#        for col in d[row].keys():
#            if not col == 'NoPrediction':
#                file.write(''.join([str(d[row][col]), '\t']))
#        file.write(''.join([str(d[row]['NoPrediction']), '\n']))




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

def createBackwardsDict(fileName,labels):
    mapping = {}
    file = getFile(fileName)
    for line in file:
        vector = line.rstrip('\n').split(',')
        value = vector[0]
        if not len(vector) == 1:
            keys = vector[1:]
            for key in keys:
                if mapping.has_key(key):
                    print 'somethings wrong, key encountered already'
                    mapping[key] = value
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
    print '\tUsage1 (Precision/Recall): GroundTruth_labelmap Prediction_labelmap labelfile binary?(yes/no) [overwrite_arg]\n'
    print '\tUsage2 (Confusion Matrix): GroundTruth_labelmap Prediction_labelmap labelfile\n'

# Returns (# times label in file1, # times label in file2, # times file1 and file2 labeled segment with label)
def compareTwoFiles(file1, file2, file3, binaryStr, overwrite):

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

        strLst = [str(dict1LabelSetSize),',',str(dict2LabelSetSize),',',str(intersectionSize),'\n']
        if overwrite:
            fileWrite = getFileWrite(''.join([label,'.out']))
        else:
            fileWrite = getFileAppend(''.join([label,'.out']))
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

def printAll(TPI):
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

def generateConfusionMatrix(file1, file2, file3):
    confusionMatrix = {}

    labels = getListOfLabels(file3)
    labelsCopy = labels
    labelsCopy.append('NoPrediction')
    labelsPlusColumn = labelsCopy
    for label in labels:
        confusionMatrix[label] = {}
        for label2 in labelsPlusColumn:
            confusionMatrix[label][label2] = 0

    truthDict = createBackwardsDict(file1, file3)
    predictedDict = createBackwardsDict(file2, file3)

    for truthKey,truthValue in truthDict.iteritems():
        if not predictedDict.has_key(truthKey):
##            print 'truthValue', confusionMatrix[truthValue]['NoPrediction']
            confusionMatrix[truthValue]['NoPrediction'] = confusionMatrix[truthValue]['NoPrediction'] + 1
        else:
            confusionMatrix[truthValue][predictedDict[truthKey]] = confusionMatrix[truthValue][predictedDict[truthKey]] + 1

    return confusionMatrix, labelsCopy

def main():
    # TPI = Truth/Precision/Intersection triplet
    if len(sys.argv) == 4:
        TPI, labels = generateConfusionMatrix(sys.argv[1], sys.argv[2], sys.argv[3])
        f = getFileWrite('CFM.txt')
        print2DDict(TPI, f, labels)
    elif len(sys.argv) == 5 or len(sys.argv) == 6:
        overwrite = False
        if len(sys.argv) == 6:
            overwrite = True
        TPI = compareTwoFiles(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], overwrite)



##    print '\nSuccessful Evaluation'
##    printTPI(TPI)
##    printPrecision(TPI)
##    printRecall(TPI)
    pass

if __name__ == '__main__':
    main()
