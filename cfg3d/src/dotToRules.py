'''
Created on Feb 5, 2012

@author: lisherwin
'''
import pprint
import sys
import itertools

def getFile(name):
    return open(name)
def getFileAppend(name):
    return open(name, 'a')
def getFileWrite(name):
    return open(name, 'w')

debug = getFileWrite('debug')

def is_number(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

allFileImproperRules = set([])
allFileProperRules = set([])

def parseToken(token):
    vect = token.split('__')
    tokenName = None
    tokenId = None
    try:
        tokenName = vect[0].lstrip()
        tokenId = vect[1]
    except:
        print 'Unable to token split'
        raise
    
    return tokenName,tokenId
    
def parseRuleLine(ruleLine):
    vect = ruleLine.rstrip(';\n').split('->')
    LHS = None
    RHS = None    
    try:
        LHS = vect[0]
        RHS = vect[1]
    except:
        print 'Unable to line split'
        raise
    return LHS, RHS

def isRuleLine(line):
    vect = line.split('->')
    return len(vect) == 2

def handleRuleLine(ruleLine, idToRule):
    LHS, RHS = parseRuleLine(ruleLine)
    LHSName, LHSId = parseToken(LHS)
    LHSUnique = ''.join([LHSName, LHSId])

    RHSName, RHSId = parseToken(RHS)
    
    if idToRule.has_key(LHSUnique):
        idToRule[LHSUnique] = ''.join([idToRule[LHSUnique],',',RHSName])
    else:
        idToRule[LHSUnique] = ''.join([LHSName,',',RHSName])
    return idToRule

def handleLine(line, idToRule):
    if isRuleLine(line):
        idToRule = handleRuleLine(line, idToRule)
    return idToRule
        
def handleFile(fileName):
    f = getFile(fileName)
    idToRule = {}
    for line in f:
        idToRule = handleLine(line, idToRule)
    return idToRule

def isImproperRule(line):
    vect = line.split(',')
    return len(vect) > 1

def uniquefyRHS(RHS):
    return list(set(RHS))

def uniquefyRule(LHS, RHS):
    uniqueRule = LHS
    
    if RHS[0].rstrip('\n') == "Terminal":
        return ''.join([LHS, ',', 'Plane'])
    else:
        for RHSElem in RHS:
            uniqueRule = ''.join([uniqueRule,',',RHSElem.rstrip('\n')]) 
        return uniqueRule

def parseImproperRuleLine(line):
    vect = line.split(',')
    return vect[0], vect[1:]

def tallyImproperRuleLine(line, tally):
    LHS, RHS = parseImproperRuleLine(line)
    
    RHS = uniquefyRHS(RHS)
    uniqueRule = uniquefyRule(LHS, RHS)
    
    if tally.has_key(uniqueRule):
        tally[uniqueRule] += 1
    else:
        tally[uniqueRule] = 1
        
    return tally

def printList(list, file):
    for elem in list:
        file.write(''.join([elem,'\n']));

def uniquefy(fileName,improperUniqueRules):
    tally = {}
    f = getFile(fileName)
    for line in f:
        if isImproperRule(line):
            tally = tallyImproperRuleLine(line, tally)
    out = getFileWrite(improperUniqueRules)
    
    rules = tally.keys()
    rules.sort(key=lambda str: str.count(','))
    for rule in rules:
        out.write(''.join([rule, '\n']))
    
    return tally
    
def mergeStrings(lst):
    return ''.join(lst)
    
def chooseBestCandidate(candidates, mergeCount):
    #returns best of candidates
    maxCount = 0
    bestCandidate = candidates[0]
    for (first, second) in candidates:
        pairMerge1 = mergeStrings([first, ',', second])
        if mergeCount.has_key(pairMerge1):
            if mergeCount[pairMerge1] > maxCount:
                maxCount = mergeCount[pairMerge1]
                bestCandidate = (first,second)
                
        pairMerge2 = mergeStrings([second, ',', first])
        if mergeCount.has_key(pairMerge2):
            if mergeCount[pairMerge2] > maxCount:
                maxCount = mergeCount[pairMerge2]
                bestCandidate = (second,first)
    
    print 'BC ', bestCandidate, ' count ', maxCount
    return bestCandidate

def replaceMerge(merge, RHS):
    RHS.remove(merge[0])
    RHS.remove(merge[1])
    
    head = [mergeStrings([merge[0], '_', merge[1]])]
    head.extend(RHS)
    
    return head

def findSecondaryCandidates(RHS):
    candidates = []
    for elem in RHS[1:]:
        candidates.append((RHS[0],elem))
    return candidates
    
def updateMergeCount(merge, mergeCount):
    mergeString = mergeStrings([merge[0], ',', merge[1]])
    if mergeCount.has_key(mergeString):
        mergeCount[mergeString] = mergeCount[mergeString] + 1
    else: 
        mergeCount[mergeString] = 1
    return mergeCount
    
def chooseMergePair(RHS, mergeCount):
    mergeCandidates = list(itertools.combinations(RHS, 2))
    firstMerge = chooseBestCandidate(mergeCandidates, mergeCount)
    
    order = [firstMerge[0], firstMerge[1]]
    RHS = replaceMerge(firstMerge, RHS)
    mergeCount = updateMergeCount(firstMerge, mergeCount)
    
    while len(RHS) != 1:
        candidates = findSecondaryCandidates(RHS)
        laterMerge = chooseBestCandidate(candidates, mergeCount)
        order.append(laterMerge[1])
        RHS = replaceMerge(laterMerge, RHS)
        mergeCount = updateMergeCount(laterMerge, mergeCount)
    
    return RHS, mergeCount

def clean(strLst):
    cleaned = []
    for str in strLst:
        cleaned.append(str.rstrip('\n'))
    return cleaned

def filterOutSingles(improperUniqueRules, nonDouble, double):
    improperUniqueRulesFile = getFile(improperUniqueRules)
    nonDoubleFile = getFileWrite(nonDouble)
    doubleFile = getFileWrite(double)
    for line in improperUniqueRulesFile:
        if len(line.split(',')) != 2:
            nonDoubleFile.write(line)
        else:
            doubleFile.write(line)
    
def removeLHSFromKeys(tally):
    newTally = {}
    for key,value in tally.iteritems():
        vect = key.split(',')
        newKey = vect[1]
        for elem in vect[2:]:
            newKey = ''.join([newKey,',',elem])
        newTally[newKey] = value
    return newTally
    
def filterOutDoubles(tally):
    newTally = {}
    tally = removeLHSFromKeys(tally)
    for key,value in tally.iteritems():
        if len(key.split(',')) == 2:
            newTally[key] = value
    return newTally

def initializeMergeCount(mergeCount, tally):
    return mergeCount

def properfy(file, mergeCount):
    readFile = getFile(file)
    finals = []
    
    for line in readFile:
#        if isNotComplex(line):
#            rule = clean(line.split(','))
#            RHS, mergeCount = chooseMergePair(rule[1:], mergeCount)
#            finals.append(mergeStrings([rule[0], ';', RHS[0]]))
        rule = clean(line.split(','))
        RHS, mergeCount = chooseMergePair(rule[1:], mergeCount)
        finals.append(mergeStrings([rule[0], ';', RHS[0]]))
    
    return finals, mergeCount

def isNotComplex(rule):
#    vect = rule.split('_')
#    return vect[0].find("Complex") == -1
    return rule.find("Complex") == -1

if __name__ == '__main__':
    
    allRules = []
    
    # Print out all rules
    for arg in sys.argv[1:]:
        idToRule = handleFile(arg)
        fileRules = [value for value in idToRule.values()]
        allRules.extend(fileRules)
    improperRules = 'improperRules'
    printList(allRules, getFileWrite(improperRules))
    
    improperUniqueRules = 'improperUniqueRules'
    tallyOut = 'tallyOut'
    # Uniquefy and collapse rules
    tally = uniquefy(improperRules,improperUniqueRules)
    improperUniqueRules2PlusRHS = 'improperUniqueRules.2PlusRHS'
    properRules1RHS = 'properRules.1RHS'
    filterOutSingles(improperUniqueRules, improperUniqueRules2PlusRHS, properRules1RHS)
    
    properRules, b = properfy(improperUniqueRules2PlusRHS, filterOutDoubles(tally))
    pprint.pprint(properRules)
    properRules2PlusRHSFile = getFileWrite('properRules.2PlusRHS')
    
    properRules2PlusRHS = set()
    for elem in properRules:
#        if isNotComplex(elem):
#            properRules2PlusRHS.add(elem)
        properRules2PlusRHS.add(elem)
    
    for elem in sorted(properRules2PlusRHS):
        elem = elem.replace('_', ',')
        properRules2PlusRHSFile.write(mergeStrings([elem, '\n']))
        
    
    
    pass
        
    
        
        
        