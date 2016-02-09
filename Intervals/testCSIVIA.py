# testcase0.py
"""
Test case 0: Tolerable-United solutions sets.
"""
# ThickBox_inv.py
from pyIbex import *
from SIVIA import *
from vibes import vibes
import math
import time

def subMove(x,y,v,p,u,h):
    xp = v*math.cos(p)
    yp = v*math.sin(p)
    vp = u[0] -0.1*v
    pp = u[1]
    
    x = x + h*xp
    y = y + h*yp
    v = v + h*vp
    p = p + h*pp
    
    return x,y,v,p

def listBoxToDraw(boxes):
    drawable=[]
    for X in boxes:
       drawable.append((X[0][0],X[0][1], X[1][0], X[1][1]))
    return drawable

if __name__ == '__main__':
    # define the precision of your 
    r = 10
    epsilon = 1
    vibes.beginDrawing()    
    X0 = IntervalVector([[-100, 100], [-100, 100]])
    Ipos = Interval(-2.5,2.5)
    
    
    # positions
    cx1=-1+Ipos
    cy1=-50+Ipos
    cx2=5+Ipos
    cy2=-50+Ipos
    cx3=2+Ipos
    cy3=-40+Ipos
    
    # vitesses
    v1=0
    v2=0
    v3=0
    p1=math.pi
    p2=0
    p3=math.pi/2
    
    # commande
    u1=[1,0]
    u2=[1,0]
    u3=[1,0]
    
    # simulation
    h=0.7
    

    vibes.beginDrawing()
    vibes.newFigure('Zone de non detection')
    vibes.setFigureProperties({'x':500, 'y':100, 'width':800, 'height':800})
    
    # # test unitaire
    # m= [[cx1 , cy1 ],[cx2, cy2],[cx3, cy3]]
    # pdc = test3(m)
    # #vibes.clearFigure()
    # SIVIA(X0, pdc, 0.5)
    # 
    # for m_ in m:
    #     vibes.drawCircle(m_[0].mid(), m_[1].mid(), 0.2, '[k]')
    # vibes.drawArrow([-15, -15], [-15, -10], 1, 'w[w]')
    # vibes.drawArrow([-15, -15], [-10, -15], 1, 'w[w]')
    
    
    for t in range(10):
        time1 = time.time()
        m= [[cx1 , cy1 ],[cx2, cy2],[cx3, cy3]]
        pdc = (m,100,True)
        vibes.clearFigure()
        box = pyIbex.SIVIAtest(X0,m,100,1,True)
        print(type(box),len(box))
        for m_ in m:
            vibes.drawCircle(m_[0].mid(), m_[1].mid(), 0.5, '[k]')
        vibes.drawArrow([-15, -15], [-15, -10], 1, 'w[w]')
        vibes.drawArrow([-15, -15], [-10, -15], 1, 'w[w]')
        vibes.drawBoxesUnion(listBoxToDraw(box),'[r]')
        cx1,cy1,v1,p1 = subMove(cx1,cy1,v1,p1,u1,h)
        cx2,cy2,v2,p2 = subMove(cx2,cy2,v2,p2,u2,h)
        cx3,cy3,v3,p3 = subMove(cx3,cy3,v3,p3,u3,h)
        print(time.time()-time1)
        time.sleep(3)

        
    vibes.endDrawing()
