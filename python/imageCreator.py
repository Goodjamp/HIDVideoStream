import sys
import numpy as np
import os
import math
#for delete not empty directory
import shutil

borderImageHeader = "#include <stdint.h>"
borderRadiusName = "const uint8_t borderRadiusDesc["
borderAngleName  = "const uint8_t borderAngleDesc["
borderLineName   = "const uint8_t borderLineDesc["

lcdSideLen = 128
MAX_ALFA = 255.0
Rl_0 = 18.0
Rl_3 = 14.0

R_0 = 18.8
R_1 = 18.0
R_2 = 14.5
R_3 = 13.5

R0x = 18
R0y = 18

K1 = (MAX_ALFA/(R_1 - R_0))
B1 = (-K1*R_0)
K2 = (MAX_ALFA/(R_2 - R_3))
B2 = (-K2*R_3)

radius = []
angle  = []
line   = []


def createLine(xStart, xStop, yStart, yStop, isHorizontal):
    print("xStart = ", xStart,", ")
    print("xStop = ", xStop, ", ")
    print("yStart = ", yStart, ", ")
    print("yStop = ", yStop, ", ")
    x  = xStart
    y  = 0 
    while x < xStop:
        y = yStart
        while y < yStop:
            yF = y
            #d = math.sqrt((float(y0) - float(yF))**2) if bool(isHorizontal) else math.sqrt((float(x0) - float(xF))**2)
            #print("d = ", d)    
            #if (d <= Rl_0) and (d > Rl_3):
            #    line.append(x)
            #    line.append(y)
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
        xF = float(x) + float(0.5)
        y = yStart
        while y < yStop:
            yF = float(y) + float(0.5)
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
f = open('./imageSource/border' + '.h', 'w')
#file format:
#- file consist from 3 part: radius, angle, line
#- radius format:
#- - first 2 bytes: Lr = size of radius part (2 bytes int little endians)
#- - next fields: Lr fields, format of fields (real order): x, y, alfa (little endian, all numbers on byte)

f.write(borderImageHeader + "\n\n")

#calculation coordinates of line part of border
createAngle(R0x, R0y, 0, R_0, 0, R_0)
createAngle(lcdSideLen - R0x, R0y, lcdSideLen - R_0, lcdSideLen, 0, R_0)
createAngle(lcdSideLen - R0x, lcdSideLen - R0y, lcdSideLen - R_0, lcdSideLen, lcdSideLen - R_0, lcdSideLen)
createAngle(R_0 - 0.5, lcdSideLen - R_0 + 0.5, 0, R_0, lcdSideLen - R_0, lcdSideLen)
#write radius part o border
index = 0
radiusPoints = len(radius)/3
print("radius points = ", radiusPoints)
f.write(borderRadiusName)
#f.write( str(int(radiusPoints) & 255) + ", " + str((int(radiusPoints) >>8) & 255) + ", " + " // start radius description" + "\n" )
f.write( str(int(len(radius)))  + "] = { // radius description \n" )
for i in radius:
    f.write( str(int(i)) + ", " )
    index = index + 1
    if index >= 3:
        f.write( "\n" )
        index = 0
f.write( "};" + "\n\n")
#write angle part o border
index = 0
anglePoints = len(angle)/2
print("angle points = ", anglePoints)
f.write(borderAngleName)
#f.write( str(int(anglePoints) & 255) + ", " + str((int(anglePoints) >>8) & 255) + ", " + " // start angle description" + "\n"  )
f.write( str(int(len(angle)))  + "] = { // angle description \n" )
for i in angle:
    f.write( str(int(i)) + ", " )
    index = index + 1
    if index >= 2:
        f.write( "\n" )
        index = 0
f.write( "};" + "\n\n")
#calculation coordinates of line part of border
createLine( Rl_0, lcdSideLen - Rl_0, 0, 4, 1)
createLine(Rl_0, lcdSideLen - Rl_0, lcdSideLen - 4, lcdSideLen, 1)
createLine(0, 4, Rl_0, lcdSideLen - Rl_0, 0)
createLine(lcdSideLen - 4, lcdSideLen, Rl_0, lcdSideLen - Rl_0, 0)
#write line part o border
index = 0
linePoints = len(line)/2
print("line points = ", linePoints)
f.write(borderLineName)
#f.write( str(int(linePoints) & 255) + ", " + str((int(linePoints) >>8) & 255) + ", "+ " // start line description" + "\n"  )
f.write( str(int(len(line)))  + "] = { // line description \n" )
for i in line:
    f.write( str(int(i)) + ", " )
    index = index + 1
    if index >= 8:
        f.write( "\n" )
        index = 0
f.write( "};" + "\n")        
print("Total size board image = ", len(line) + len(angle) + len(radius))        
    
f.close()
sys.exit()
