import sys
import numpy as np
import os
import math
#for delete not empty directory
import shutil

borderImageHeader = "#include <stdint.h>"
borderImageName = "const uint8_t borderImageDesc[] = {"

lcdSideLen = 128
MAX_ALFA = 255.0
R_0 = 18.0
R_1 = 17.0
R_2 = 15.0
R_3 = 14.0

K1 = (MAX_ALFA/(R_1 - R_0))
B1 = (-K1*R_0)
K2 = (MAX_ALFA/(R_2 - R_3))
B2 = (-K2*R_3)

radius = []
angle  = []
line   = []


def createLine(x0, y0, xStart, xStop, yStart, yStop, isHorizontal):
    x  = xStart
    y  = 0 
    xF = 0.0
    yF = 0.0
    d  = 0.0
    while x < xStop:
        xF = x + 0.5
        y = yStart
        
        while y < yStop:
            yF = y + 0.5
            d = math.sqrt((float(y0) - float(yF))**2) if bool(isHorizontal) else math.sqrt((float(x0) - float(xF))**2)
          
            #if bool(isHorizontal):
            #    d = math.sqrt((float(y0) - float(yF))**2)
            #else:
            #    d = math.sqrt((float(x0) - float(xF))**2)
                
            if (d < R_0) and (d > R_3):
                line.append(x)
                line.append(y)
            y = y + 1
            
        x = x + 1


def createAngle(x0In, y0In, xStart, xStop, yStart, yStop):
    x = 0
    y = 0
    xF = 0.0
    yF = 0.0
    d = 0.0
    x = xStart
    while x < xStop:
        xF = x + 0.5
        y = yStart
        while y < yStop:
            yF = y + 0.5
            d = math.sqrt((float(xF) - float(x0In))**2 + (float(yF) - float(y0In))**2);

            if d > R_0:
                angle.append(x)
                angle.append(y)
            if (d < R_0) and (d > R_1):
                alfa = d * K1 + B1
                radius.append(x)
                radius.append(y)
                radius.append(alfa)
            if (d < R_1) and (d > R_2):
                radius.append(x)
                radius.append(y)
                radius.append(MAX_ALFA)
            if (d < R_2) and (d > R_3):
                alfa = d * K2 + B2
                radius.append(x)
                radius.append(y)
                radius.append(alfa)
            y = y + 1       
        x = x + 1



        
try:
    if not os.path.exists('imageSource'):
        os.makedirs('imageSource')
    else:
        print("delet dir")
        shutil.rmtree("imageSource")    
        os.makedirs('imageSource')
except OSError:
    print ('Error: Creating directory imageSource')

# save  border data
createLine(1,2,3,4,5,6, 1)
f = open('./imageSource/border' + '.c', 'w')
#file format:
#- file consist from 3 part: radius, angle, line
#- radius format:
#- - first 2 bytes: Lr = size of radius part (2 bytes int little endians)
#- - next fields: Lr fields, format of fields (real order): x, y, alfa (little endian, all numbers on byte)

f.write( borderImageHeader + "\n\n" + borderImageName)

#calculation coordinates of line part of border
createAngle(R_0, R_0, 0, R_0, 0, R_0)
createAngle(lcdSideLen - R_0, R_0, lcdSideLen - R_0, lcdSideLen, 0, R_0)
createAngle(lcdSideLen - R_0, lcdSideLen - R_0, lcdSideLen - R_0, lcdSideLen, lcdSideLen - R_0, lcdSideLen)
createAngle(R_0, lcdSideLen - R_0, 0, R_0, lcdSideLen - R_0, lcdSideLen)
#write radius part o border
index = 0
print("radius len = ", len(radius))
f.write( str(len(radius) & 255) + ", " + str((len(radius) >>8) & 255) + ", " + " // start radius description" + "\n" )
for i in radius:
    f.write( str(int(i)) + ", " )
    index = index + 1
    if index >= 3:
        f.write( "\n" )
        index = 0

#write angle part o border
index = 0
print("angle len = ", len(angle))
f.write( str(len(angle) & 255) + ", " + str((len(angle) >>8) & 255) + ", " + " // start angle description" + "\n"  )
for i in angle:
    f.write( str(int(i)) + ", " )
    index = index + 1
    if index >= 2:
        f.write( "\n" )
        index = 0

#calculation coordinates of line part of border
createLine(R_0, R_0, R_0, lcdSideLen - R_0, 0, R_0, 1)
createLine(R_0, lcdSideLen - R_0, R_0, lcdSideLen - R_0, lcdSideLen - R_0, lcdSideLen, 1)
createLine(R_0, R_0, 0, R_0, R_0, lcdSideLen - R_0, 0)
createLine(lcdSideLen - R_0, R_0, lcdSideLen - R_0, lcdSideLen, R_0, lcdSideLen - R_0, 0)
#write line part o border
index = 0
print("line len = ", len(line))
f.write( str(len(line) & 255) + ", " + str((len(line) >>8) & 255) + ", "+ " // start line description" + "\n"  )
for i in line:
    f.write( str(int(i)) + ", " )
    index = index + 1
    if index >= 8:
        f.write( "\n" )
        index = 0
print("Total size board image = ", len(line) + len(angle) + len(radius))        
    
f.write( "};" )
f.close()
sys.exit()
