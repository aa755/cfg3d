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
#    file.write(','.join(labels))
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

def nodesHavingLabel(fileName,labels):
    nodeset = set([])
    file = getFile(fileName)
    for line in file:
        vector = line.rstrip('\n').split(',')
        key = vector[0]
        assert(not len(vector) == 1) # why failing
        if (not len(vector) == 1) and (key in labels) :
            value = vector[1:]
            nodeset.update(set(value))
    return nodeset

def createBackwardsDict(fileName,labels):
    mapping = {}
    file = getFile(fileName)
    count = 0
    for line in file:
        vector = line.rstrip('\n').split(',')
        value = vector[0]
        if not len(vector) == 1:
            keys = vector[1:]
            count = count + len(keys)
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
    gt = createDict(file1,labels)
    pred = createDict(file2,labels)
    gtLabeledList = nodesHavingLabel(file1, labels)

    totalDict1 = 0
    totalDict2 = 0
    totalIntersections = 0
    for label in labels:
        dict1LabelSet = set([])
        dict2LabelSet = set([])
        if gt.has_key(label):
            dict1LabelSet = set(gt[label]) #set of nodes having that label in gt
        if pred.has_key(label):
            dict2LabelSet = set(pred[label])

        dict1LabelSetSize = len(dict1LabelSet);
        dict2LabelSetSize = len(dict2LabelSet.intersection(gtLabeledList))
        intersectionSize = len(dict1LabelSet.intersection(dict2LabelSet))

        assert(binary==False)
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

def count(dict):
    count = 0
    for key,value in dict.iteritems():
        for key1,value1 in value.iteritems():
            count = count + value1
    return count

def dictMax(dict1, dict2):
    maxSoFar = 0
    for key1,value in dict1.iteritems():
        key = int(key1)
        if key > maxSoFar:
            maxSoFar = key
            
    for key2,value in dict2.iteritems():
        key = int(key2)
        if key > maxSoFar:
            maxSoFar = key
            
    return maxSoFar

def generateConfusionMatrix(file1, file2, file3):
    confusionMatrix = {}

    labels = getListOfLabels(file3)
    labelsCopy = labels
    labelsCopy.append('other')
    labelsPlusColumn = labelsCopy
    
    for label in labelsPlusColumn:
        confusionMatrix[label] = {}
        for label2 in labelsPlusColumn:
            confusionMatrix[label][label2] = 0

    print 'truth'
    truthSegToLabel = createBackwardsDict(file1, file3)
    truthLabelToSegs = createDict(file1, file3)
#    pprint.pprint(truthDict)
    print 'predicted'
    predictedSegToLabel = createBackwardsDict(file2, file3)
    predictedLabelToSegs = createDict(file2, file3)
#    pprint.pprint(predictedDict)
    
    
    for j in range(1,int(dictMax(truthSegToLabel, predictedSegToLabel))+1):
        i = str(j)
        if truthSegToLabel.has_key(i) and not predictedSegToLabel.has_key(i):
            if truthSegToLabel[i] in labels:
                confusionMatrix[truthSegToLabel[i]]['other'] += 1
        elif not truthSegToLabel.has_key(i) and predictedSegToLabel.has_key(i):
            if predictedSegToLabel[i] in labels:
                confusionMatrix['other'][predictedSegToLabel[i]] += 1
        elif truthSegToLabel.has_key(i) and predictedSegToLabel.has_key(i):
            if predictedSegToLabel[i] in labels and truthSegToLabel[i] in labels:
                confusionMatrix[truthSegToLabel[i]][predictedSegToLabel[i]] += 1
            
            elif predictedSegToLabel[i] in labels and not truthSegToLabel[i] in labels:            
                confusionMatrix['other'][predictedSegToLabel[i]] += 1
                
            elif not predictedSegToLabel[i] in labels and truthSegToLabel[i] in labels:
                confusionMatrix[truthSegToLabel[i]]['other'] += 1
    
    return confusionMatrix, labelsCopy




def main():
    # TPI = Truth/Precision/Intersection triplet
    if len(sys.argv) == 4:
        TPI, labels = generateConfusionMatrix(sys.argv[1], sys.argv[2], sys.argv[3])
        f = getFileAppend('CFM.txt')
        print2DDict(TPI, f, labels)
        print 'count = ', count(TPI)
        pprint.pprint(TPI)
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
