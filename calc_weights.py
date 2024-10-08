import math
import random

def sigmoid(x):
    return 1/(1+math.exp(-x))

def Perceptron(u1, w1, u2, w2):
    return sigmoid((u1 * w1) + (u2 * w2))

def deltaWeights(mu, ui, v, t):
    return mu * v * (1-v) * (t-v) * ui

mu = 0.1
#w1 = random.random()
#w2 = random.random()

print("test: " + str(Perceptron(0.5, -0.8, 1, 0.8)))
print("Delta: " + str(deltaWeights(5, 0.5, Perceptron(0.5, -0.8, 1, 0.8), 1)))

w1 = 0.2
w2 = 0.2

#u1 = 1
#u2 = 0

#t = 0

file = open("data.txt", "r")

#

#for x in range(len(numbers)):
#    print(numbers[x])

#print(float(numbers[0]) + float(numbers[2]))
#print(file.read())




for x in range(3000):
    line = file.readline()
    numbers = line.split()
    
    u1 = float(numbers[0])
    u2 = float(numbers[1])
    t = float(numbers[2])

    w1 = w1 + deltaWeights(mu, u1, Perceptron(u1, w1, u2, w2), t)
    w2 = w2 + deltaWeights(mu, u2, Perceptron(u1, w1, u2, w2), t)
    #print("weight1: " + str(w1) + ", weight2: " + str(w2))

print(w1)
print("weight1: " + str(w1) + ", weight2: " + str(w2))



#print(w1)

#file = open("data.txt", "r")

#line = file.readline()

#numbers = line.split()

#for x in range(len(numbers)):
#    print(numbers[x])

#print(float(numbers[0]) + float(numbers[2]))
#print(file.read())

file.close();