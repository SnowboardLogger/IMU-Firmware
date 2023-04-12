from vpython import *
x = box(length=6, width=2, height=0.2, color=color.white);
y=cylinder(length=6, radiud=0.5, color=color.yellow);
y.pos=vector(-3, 0, 0)
j=0
change=1
while True:
    rate(20)
    j=j+change
    y.length=j
    if(j==10):
        change=j=-1
    if(j==1):
        change=1
