'''
Created on Feb 5, 2012

@author: lisherwin
'''

import pprint
import sys
import itertools
import pickle

def getFile(name):
    return open(name)
def getFileAppend(name):
    return open(name, 'a')
def getFileWrite(name):
    return open(name, 'w')
def mergeStrings(lst):
    return ''.join(lst)

functionOpen = 'void appendLearningRules(vector<RulePtr>& learningRules) {\n'
temp = '\tvector<string> temp;\n\tvector<double> tempratio;\n'
functionClose = '}\n'

def printBeforeAppend(dataStructuresFile):
    dataStructuresFile.write('\ntemplate<typename SupportType, typename RHS_Type2 >\n')
    dataStructuresFile.write('bool DoubleRuleComplex<SupportType,RHS_Type2> :: canBaseBeHallucinated()\n')
    dataStructuresFile.write('{\n')
    dataStructuresFile.write('return typeid(Floor)==typeid(SupportType); // should be replaced by static comparison\n')
    dataStructuresFile.write('}\n\n')
    

def printImports(dataStructuresFile):
    dataStructuresFile.write('#include "structures.h"\n\n')

def isSupportElem(supportElem, supportedElem):
    return supportElem.replace('Complex','') == supportedElem 

###
def isImproperRule(line):
    vect = line.split(',')
    return len(vect) > 1

def uniquefyRHS(RHS):
    return sorted(RHS)

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

def getRuleRatio(line, ratios):
    LHS, RHS = parseImproperRuleLine(line)
    
    RHS = uniquefyRHS(RHS)
    uniqueRule = uniquefyRule(LHS, RHS)
    return str(ratios[uniqueRule])
###

def printSingleRuleNTs(singleRuleFile, dataStructuresFile, addingSingleRuleText, ratios):
    currentDeclarations = set()
    currentRules = set()
    for line in singleRuleFile:
        vect = line.split(',')
        if not isNotComplex(line):
            #print vect[0], vect[1]
            
            LHS = vect[0]
            LHS = LHS.replace('Complex','')
            #LHS = LHS[:-7]
            #print LHS
            RHS = vect[1].rstrip('\n')
            complexDeclaration = mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new SingleRuleComplex<',LHS,'>()));\n'])
            if not complexDeclaration in currentRules:     
                addingSingleRuleText = mergeStrings([addingSingleRuleText, complexDeclaration])
                currentRules.add(complexDeclaration)
            
            addRule = mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<',LHS,',',RHS,'>(temp)));\n'])
            if not addRule in currentRules:
                clear = '\ttemp.clear();\n'
                if not isNotComplex(RHS):
                    RHS = mergeStrings(['SupportComplex<',RHS.replace('Complex',''),'>'])
                    
                if not isSupportElem(LHS, RHS):
                    pushBack = mergeStrings(['\ttemp.push_back(typeid(',RHS,').name());\n'])
                    addingSingleRuleText = mergeStrings([addingSingleRuleText, clear, pushBack, addRule])
                    currentRules.add(addRule)
        elif vect[1] == 'Plane\n':
            declaration = mergeStrings(['class ', vect[0], ' : public PlanarPrimitive{};\n'])
            
            if not declaration in currentDeclarations:
                dataStructuresFile.write(declaration)
                currentDeclarations.add(declaration)
            
            addRule = mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new SingleRule<', vect[0], ',Plane>()));\n'])
            addingSingleRuleText = mergeStrings([addingSingleRuleText, addRule])
        else:
            declaration = mergeStrings(['class ', vect[0], ' : public NonTerminal{};\n'])
            
            if not declaration in currentDeclarations:
                dataStructuresFile.write(declaration)
                currentDeclarations.add(declaration)
            ratio = getRuleRatio(line.replace(';', ',').rstrip('\n'), ratios)
            addRule = mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new SingleRuleNoFeature<', vect[0], ',',vect[1].rstrip('\n'),'>(', ratio, ')));\n'])
            addingSingleRuleText = mergeStrings([addingSingleRuleText, addRule])
    return addingSingleRuleText, currentDeclarations, currentRules

def doubleRuleString(one, two, three):
    return mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new DoubleRule<', one,',', two, ',', three, '>()));\n'])

def replaceMerge(merge, RHS):
    RHS.remove(merge[0])
    RHS.remove(merge[1])
    
    head = [mergeStrings([merge[0], '_', merge[1]])]
    head.extend(RHS)
    
    return head

def generateNTIs(RHS,dataStructuresFile, addingDoubleRuleText, currentDeclarations, currentRules):
    while len(RHS) >= 2:
        NT = mergeStrings([RHS[0],'_',RHS[1].rstrip('\n')])
        
        declaration = mergeStrings(['class ', NT, ' : public NonTerminalIntermediate{};\n'])
        if not declaration in currentDeclarations:
            dataStructuresFile.write(declaration)
            currentDeclarations.add(declaration)
            
        newRule = doubleRuleString(NT, RHS[0], RHS[1].rstrip('\n'))
        if not newRule in currentRules:
            addingDoubleRuleText = mergeStrings([addingDoubleRuleText, newRule])
            currentRules.add(newRule)
        
        RHS = replaceMerge([RHS[0], RHS[1]], RHS)
    return RHS, addingDoubleRuleText, currentDeclarations, currentRules

def handleDoubleRuleNTLine(line, dataStructuresFile, addingDoubleRuleText, currentDeclarations, currentRules):
    LHS,RHS = line.split(';')
    RHS = RHS.split(',')
    RHS, addingDoubleRuleText, currentDeclarations, currentRules = generateNTIs(RHS, dataStructuresFile, addingDoubleRuleText, currentDeclarations, currentRules)
    #declaration = mergeStrings(['\nclass ', LHS, '_', RHS[0].rstrip('\n'), ' : public NonTerminal{};\n'])
    declaration = mergeStrings(['class ', LHS, ' : public NonTerminal{};\n'])
    if not declaration in currentDeclarations:
        dataStructuresFile.write(declaration)
        currentDeclarations.add(declaration)
    return addingDoubleRuleText, currentDeclarations, currentRules

def isNotComplex(rule):
    return rule.find("Complex") == -1

def isGood(rule):
    return rule.find("Occluded") == -1 and rule.find("WallComplexH") == -1 

def printDoubleRuleNTs(doubleRuleFile, dataStructuresFile, addingDoubleRuleText, currentDeclarations, currentRules, complexToElems, ratios):
    for line in doubleRuleFile:
        if isGood(line):
            if isNotComplex(line):
                addingDoubleRuleText, currentDeclarations, currentRules = handleDoubleRuleNTLine(line, dataStructuresFile, addingDoubleRuleText, currentDeclarations, currentRules)
                LHS,RHS = line.split(';')
                RHS = RHS.split(',')
#                addRule = doubleRuleString(LHS, RHS[0], RHS[1].rstrip('\n'))
                ratio = getRuleRatio(line.replace(';',',').rstrip('\n'), ratios)
                addRule = mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new SingleRuleNoFeature<',LHS,',','_'.join(RHS).rstrip('\n'),'>(', ratio, ')));\n'])
                if not addRule in currentRules:     
                    addingDoubleRuleText = mergeStrings([addingDoubleRuleText, addRule])
                    currentRules.add(addRule)
            else:
                LHS,RHS = line.split(';')
                
                LHSSupport = LHS.replace('Complex','')
                
                complexDeclaration = mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new SingleRuleComplex<',LHSSupport,'>()));\n'])
                if not complexDeclaration in currentRules:     
                    addingDoubleRuleText = mergeStrings([addingDoubleRuleText, complexDeclaration])
                    currentRules.add(complexDeclaration)
                
                complexVect = complexToElems[LHS]                
                buildVect = '\ttemp.clear();\n\ttempratio.clear();\n'
                buildVect = mergeStrings([buildVect, complexVect])
                tempInitialized = False
                
                for elem in RHS.split(','):
                    if not isNotComplex(elem):
                        elem = mergeStrings(['SupportComplex<',elem.replace('Complex', '').rstrip('\n'),'> '])
                    if not isSupportElem(LHS,elem.rstrip('\n')):
                        addRule = mergeStrings(['\tappendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<',LHSSupport,',',elem.rstrip('\n'),'>(temp)));\n'])
                        if not addRule in currentRules:
                            if not tempInitialized:
                                addingDoubleRuleText = mergeStrings([addingDoubleRuleText, buildVect, addRule])
                                # print 'builtVect'
                                tempInitialized = True
                            else:
                                # print 'did not build vect'
                                addingDoubleRuleText = mergeStrings([addingDoubleRuleText, addRule])
                                tempInitialized = True
                        currentRules.add(addRule)
    return addingDoubleRuleText

def complexRuleParse(file, ratios):
    complexToElems = {}
    buildingVectString = ''
    for line in file:
        LHS,RHS = line.split(';')
        supported = map(lambda x: x.rstrip('\n'), RHS.split(','))
        for supportedElem in supported:
            if isNotComplex(supportedElem) and not isSupportElem(LHS, supportedElem):                
                buildingVectString = ''.join([buildingVectString, '\ttemp.push_back(typeid(',supportedElem,').name());\n'])
                buildingVectString = ''.join([buildingVectString, '\ttempratio.push_back(',str(ratios[''.join([LHS,'Objects,',supportedElem])]),');\n'])
            else:
                if isGood(supportedElem):
                    buildingVectString = ''.join([buildingVectString, '\ttemp.push_back(typeid(SupportComplex<',supportedElem.replace('Complex',''),'>).name());\n'])
                    buildingVectString = ''.join([buildingVectString, '\ttempratio.push_back(',str(ratios[''.join([LHS,'Objects,',supportedElem])]),');\n'])            
            #print buildingVectString
        complexToElems[LHS] = buildingVectString  
        buildingVectString = ''
    return complexToElems

if __name__ == '__main__':
    #list1 = ['chairBack','CPUSide','CPUFront']
    ##list2 = ['CPUFront','chairBack','CPUSide']
    #print set(list1)
    #print set(list2)
    ratios = pickle.load(getFile('ratios.pickle'))
    pprint.pprint(ratios)
    doubleRuleFile = getFile('properRules.2PlusRHS')
    singleRuleFile = getFile('properRules.1RHS')
    complexRuleFile = getFile('complex')
    complexToElems = complexRuleParse(complexRuleFile,ratios)
    
    dataStructuresFile = getFileWrite('generatedDataStructures.cpp')
    
    printImports(dataStructuresFile)
    
    addingSingleRuleText = ''
    addingDoubleRuleText = ''
    addingSingleRuleText, currentDeclarations, currentRules = printSingleRuleNTs(singleRuleFile, dataStructuresFile, addingSingleRuleText, ratios)
    addingDoubleRuleText = printDoubleRuleNTs(doubleRuleFile, dataStructuresFile, addingDoubleRuleText, currentDeclarations, currentRules, complexToElems, ratios)
    
    printBeforeAppend(dataStructuresFile)
    dataStructuresFile.write(functionOpen)   
    dataStructuresFile.write(temp)   
    dataStructuresFile.write(addingSingleRuleText)
    dataStructuresFile.write(addingDoubleRuleText) 
    dataStructuresFile.write(functionClose)

    pass