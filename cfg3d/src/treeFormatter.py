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
def f(string):
    return string.rstrip('\n').rstrip(' ').rstrip(' ;')

def type(str):
    vect = str.split('I5')
    support = None
    if len(vect)==2:
        vect = vect[1].split('E__')
        type = vect[0]
        return type
    else:
        return None

def handleLine(line, d):
    vect = line.split(' -> ')
    if len(vect) == 2:
        LHS = f(vect[0])
        RHS = f(vect[1])
        if d.has_key(LHS):
            d[LHS].append(RHS)
        else:
            d[LHS] = [RHS]
    return d

def cleanDict(d):
    encountered = set()
    flag = True
    newD = {}
    while flag:
        flag = False
        for LHS,RHS in d.iteritems():
            newRHS = [] 
            if not type(LHS) == None: # if is a support
                for RHSE in RHS:
                    if not type(RHSE) == type(LHS):
                        newRHS.append(RHSE)
                    else:
                        flag = True
                        encountered.add(RHSE)
                        newRHS.extend(d[RHSE])
                newD[LHS] = newRHS
            else: 
                newD[LHS] = RHS
        d = newD
    return newD, encountered

def dictToDot(d,file,encountered):
    file.write('digraph g{\n')
    for k,v in d.iteritems():
        if not k in encountered:
            for elem in v:
                file.write(''.join([k,' -> ',elem,' ;\n']))
    file.write('}')        

def handleFile(fileName):
    f = getFile(fileName)
    d = {}
    for line in f:
        d = handleLine(line, d)
    return d
    

if __name__ == '__main__':
    fileName = sys.argv[1]
    d = handleFile(fileName)
    newD,encountered = cleanDict(d)
    dictToDot(newD,getFileWrite(sys.argv[1]),encountered)
#    str = 'SupportComplexI5FloorE__15722__106'
    pass
