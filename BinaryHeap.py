# -*- coding: utf-8 -*-
"""
Created on Tue Jul  7 19:27:21 2020

@author: Harry
"""

class Heap:
    #constructor
    def __init__(self):
        self.list = [0]
        self.size = 0
    
    #inserts desired value into heap and appropriately orders the priority
    def insert(self,n):
        self.list.append(n)
        self.size += 1
        
        j = self.size
        while j // 2 > 0:
            k = self.list[j // 2]
            if self.list[j] < k:
                self.list[j // 2] = self.list[j]
                self.list[j] = k
            j //= 2
                
            
    def delete(self):
        retVal = self.list[1]
        self.list[1] = self.list[self.size]
        self.size -= 1
        self.list.pop()
        i = 1
        while (i * 2) <= self.size:
            lastChild = self.Child(i)
            #switch position if current node has bigger f or has same f but smaller g(ie tie break)
            if self.list[i].f > self.list[lastChild].f or ((self.list[i].f == self.list[lastChild].f) and (self.list[i].g < self.list[lastChild].g)):
                temp = self.list[i]
                self.list[i] = self.list[lastChild]
                self.list[lastChild] = temp
            i = lastChild
        return retVal

  
    def Child(self,i):
        if i * 2 + 1 > self.size:
            return i * 2
        else:
            if self.list[i*2].f >= self.list[i*2+1].f:
                return i * 2 + 1
            else:
                return i * 2
    
    def printHeap(self):
        i = 1;
        while(i <= self.size):
            if self.size == 0:
                print("empty list")
            else:
                print(self.list[i])
            #print("i inside printHeap is " + str(i))
            i += 1
    

