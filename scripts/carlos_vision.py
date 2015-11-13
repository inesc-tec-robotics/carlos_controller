#!/usr/bin/env python

"""
Created on Thu Jun 12 12:45:53 2014

@author: Iago Vaamonde
"""

import cv2
import math

import numpy as np
from studs_defines import WMODE_STOPPED
from studs_defines import WMODE_DETECT
from studs_defines import WMODE_TRACK


from operator import itemgetter

CAMARA=0


##Colores
AZUL    =[255,0,0]
VERDE   =[0,255,0]
ROJO    =[0,0,255]

AMARILLO=[0,255,255]
MAGENTA =[255,0,255]
CIAN    =[255,255,0]

BLANCO  =[255,255,255]
NEGRO   =[0,0,0]

AZUL_OSCURO    =[127,0,0]
VERDE_OSCURO   =[0,127,0]
ROJO_OSCURO    =[0,0,127]




def nothing(*arg):
    pass

def coordenadas_punto_ventana(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
##    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(vis2,(x,y),9,ROJO,9)
        x1,y1,z1=calcula_coordenadas_3D(x,y)
        print x1,y1,z1

def calcula_distancia(img, th):
    b, g, r = cv2.split(img)
    canal=r
    r2 = canal.copy()
    alto_img, ancho_img, canales_img= img.shape
    H_img=alto_img/2

    ##segmentacion del laser
    r2[canal < th] = 0
    
    ##elimina puntos aislados
    kernel_dil=np.mat('1;1',np.uint8)
    r3=cv2.dilate(r2,kernel_dil)

    ##cierre para unir la linea horizontalmente
    kernel_ero=np.mat('1 1 1 1 1 1 1 1 1 1 1 1',np.uint8)
    r4=cv2.erode(r3,kernel_ero)

    kernel_dil=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(8,4))
    perfil_laser=cv2.dilate(r4,kernel_dil)



    ##seleccion de contornos
    contours, hierarchy = cv2.findContours(perfil_laser,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    ang_predominante=0
    Lmax=0
    puntos_extremos=[]
    for n in range(len(contours)):
        if len(contours[n])>4:
            cont=contours[n]
            rect=cv2.minAreaRect(contours[n])
            (xrect,yrect),(eM,em),ang=rect
            if eM>em:
                L=eM/2
                l=em/2
            else:
                l=eM/2
                L=em/2
                ang=ang+90

                

            
            if L/l>4:
                cv2.drawContours(perfil_laser, contours, n, 255, 2)
                leftmost = tuple(cont[cont[:,:,0].argmin()][0])
                rightmost = tuple(cont[cont[:,:,0].argmax()][0])
                if leftmost[0]>2:
                    puntos_extremos.append(leftmost)
                if rightmost[0]<ancho_img-2:
                    puntos_extremos.append(rightmost)
                if L>Lmax:
                    Lmax=L
                    ang_predominante=ang

            else:
                cv2.drawContours(perfil_laser, contours, n, 40, 1)



   ##medicion de la altura de la linea
    dist_p=np.zeros(ancho_img,np.int16)
    dist_m=np.zeros(ancho_img,np.int16)
    dist_m_der=np.zeros(ancho_img,np.int16)
      
    
    
    
    for x in range(ancho_img):
        borde_laser_sup=0
        borde_laser_inf=0
        intensidad_laser=0
        y_laser=0
        centro_laser=0
        for y in range(alto_img):
            if perfil_laser.item(y,x)>0:
                if borde_laser_inf==0:
                    borde_laser_inf=y
            else:
                if borde_laser_sup==0 and borde_laser_inf!=0:
                    borde_laser_sup=y
                    centro_laser=(borde_laser_sup+borde_laser_inf)/2
                    borde_laser_sup=0
                    borde_laser_inf=0
                    ##si hay varios selecciona el punto de mayor intensidad
                    if perfil_laser.item(centro_laser,x)>intensidad_laser:
                        intensidad_laser=perfil_laser.item(centro_laser,x)
                        y_laser=centro_laser



        dist_p.itemset(x,y_laser-H_img)
            


    ##calibracion pixel-metro

#    print dist_p.item(320)
#    x1_cal=0.725 #distancias en metros
#    x2_cal=0.980
#
#    p1_cal=89 #altura p en pixeles desde el eje central (dist_p)#    
#    p2_cal=157

#    c2=((1/x2_cal)-(1/x1_cal))/(p1_cal-p2_cal)
#    c1=1/x1_cal+c2*p1_cal
#    print c1
#    print c2
    
    ##transformacion pixeles-metros

    c1=1.849 #2.184367
    c2=0.00528 #0.005400



                ## ecuacion distancia:
                ##
                ## x=k1/(k2-tan(k3*p))
                ##
                ## x: distancia en metros
                ## p: distancia en pixeles al centro de la imagen
                ##
                ## aproximando para angulos pequenos tan(a)=k*a:

    dist_m=1000/(c1-c2*dist_p) 


    ##Derivada de la distancia
    distancia_der_ant=0
    for i in range(1,ancho_img):
        if dist_p.item(i)!=-H_img:
            dist_m_der.itemset(i,10*dist_m.item(i)-distancia_der_ant)
            distancia_der_ant=10*dist_m.item(i)

        else:
            dist_m_der.itemset(i,dist_m_der.item(i-1))
    	

    ang_predominante=ang_predominante/180*math.pi        

    return dist_m, dist_p, perfil_laser, puntos_extremos, ang_predominante


def calcula_profundidad (x, y):
    return x

def calcula_coordenadas_3D(x_img, y_img):
    
    global ancho_img

    factor_escala=1.1874
    distancia=distancias_mm[x_img]
    escala_x=factor_escala*distancia/ancho_img
    escala_y=escala_x

    x_img=x_img-ancho_img/2
    y_img=y_img-alto_img/2
    
    x=x_img*escala_x
    y=y_img*escala_y
    z=distancia

    return x,y,z

def calcula_coordenadas_imagen(x, y, z):

    factor_escala=1.1874
    global ancho_img
    escala_x=factor_escala*z/ancho_img
    escala_y=escala_x

    x_img=x/escala_x
    y_img=y/escala_y


    x_img=int(x_img+ancho_img/2)
    y_img=int(y_img+alto_img/2)
    

    return x_img, y_img


    

def calcula_puntos_en_rectangulo(psd, psi, pii, pid, roll, pitch, yaw):

    global patron_puntos
    global patron_margin
    global patron_prox

    ny=len(patron_puntos)
    nx=len(patron_puntos[0])
    
##    print nx,ny
    num_puntos = sum(1 for i in patron_puntos.flat if i)
    puntos = [[0 for x in xrange(num_puntos)] for x in xrange(num_puntos)] 

    x1,y1,z1=psd
    x2,y2,z2=psi
    x3,y3,z3=pii
    x4,y4,z4=pid

    patron_margin=0.05

    x1=x1-patron_margin*1000
    x4=x4-patron_margin*1000
    x2=x2+patron_margin*1000
    x3=x3+patron_margin*1000
               

    lx=(x1-x2)/(nx-1)
    ly=(y1-y4)/(ny-1)
    lzx=(z1-z2)/(nx-1)
    lzy=(z1-z4)/(ny-1)
    n=0
    
    euler_a_cuaternio(roll,pitch,yaw)    
    
    for i in range(ny):
        for j in range(nx):
            if patron_puntos[i][j]==1:

                puntos[n]=[[x1-j*lx,y4+i*ly,z1-j*lzx+i*lzy],euler_a_cuaternio(roll,pitch,yaw)]
                n=n+1

    return puntos

def euler_a_cuaternio(roll, pitch, yaw):
    
    global representa    
    
    if (representa==1):
        quaternion = [roll, pitch, yaw]
    
    else:
        import tf
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        
   
    return quaternion 
    

def dibuja_pinchos_en_rectangulo(psd, psi, pii, pid):
    
    global vis2
    global patron_puntos

    ny=len(patron_puntos)
    nx=len(patron_puntos[0])
    
##    print nx,ny
    num_puntos = sum(1 for i in patron_puntos.flat if i)
    puntos = [[0 for x in xrange(num_puntos)] for x in xrange(num_puntos)] 

    x1,y1=psd
    x2,y2=psi
    x3,y3=pii
    x4,y4=pid

    lx=(x1-x2)/(nx-1)
    ly=(y1-y4)/(ny-1)
    n=0
#    for i in range(ny):
#        for j in range(nx):
#            if patron_puntos[i][j]==1:
#                n=n+1
#                cv2.circle(vis2,(x1-j*lx,y4+i*ly),4,AZUL,2)

    return puntos

def cambia_patron(pat, margin, prox):

    global patron_puntos
    global patron_margin
    global patron_prox

    patron_margin=margin
    patron_prox=prox
    patron_puntos=np.array(pat)

def arranca():
    
    global captura
    global alto_img
    global ancho_img
    global patron_puntos
    global representa
    
    representa=0

    captura=cv2.VideoCapture(CAMARA)

    flag, img = captura.read()
    alto_img, ancho_img, canales_img= img.shape


#     patron_puntos=np.array([[0,0,0,0,0,0,0],
#                            [0,0,1,0,1,0,0],
#                            [0,1,0,1,0,1,0],
#                            [0,1,0,0,0,1,0],
#                            [0,1,0,0,0,1,0],
#                            [0,0,1,0,1,0,0],
#                            [0,0,0,1,0,0,0],
#                            [0,0,0,0,0,0,0]])

    patron_puntos=np.array([[0,0,0,0,0,0,0],[0,1,0,0,0,1,0],[0,0,0,1,0,0,0],[0,1,0,0,0,1,0],[0,0,0,0,0,0,0]])
                         
    patron_puntos = np.flipud(patron_puntos)
    patron_puntos = np.fliplr(patron_puntos)


    


def calcula_dist(modo, thres_laser):
    
    result=[[[0,0,0],[2,1,1,1]]]
    ref1=0
    ref2=0


    global distancias_mm
    global vis2
    global captura
    global alto_img
    global ancho_img
    global representa
    
    
 
    
    try: captura
    except: arranca()
    
    flag, img = captura.read()

    alto_img, ancho_img, canales_img= img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    
    th=190

    th=thres_laser

    if (representa==1):    
        th=cv2.getTrackbarPos('th', 'laser')

    distancias_mm,distancias_pixeles,img_perfil_laser, puntos_de_borde, ang_laser=calcula_distancia(img,th)


    ## umbral adaptativo para detectar los bordes
    tam_filtro= 15
    c_filtro= 2
    edge = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,tam_filtro,c_filtro)
                
    edgeh=edge.copy()    
    edge_or=edge.copy()        

    ## dilatacion vertical + erosion, bordes verticales
    kernel_ero=np.ones((15,1),np.uint8)
    edge=cv2.dilate(edge,kernel_ero)
    
    kernel_dil=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,38))
    edge=cv2.erode(edge,kernel_dil)

    ## dilatacion horizontal + erosion, bordes horizontales
    kernel_ero=np.ones(10,np.uint8)
    edgeh=cv2.dilate(edgeh,kernel_ero)
    
    kernel_dil=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    edgeh=cv2.erode(edgeh,kernel_dil)

    
    ## marco para que los bordes no toquen el borde de la imagen
    ancho_borde=10
    edge_red=np.zeros((alto_img-2*ancho_borde,ancho_img-2*ancho_borde),np.uint8)
    edgebor= cv2.copyMakeBorder(edge_red,ancho_borde,ancho_borde,ancho_borde,ancho_borde,cv2.BORDER_CONSTANT,value=AZUL)
    edge2=edgebor+edge
    edge2h=edgebor+edgeh
    
    ## extraccion de contornos. vertical y horizontal
    contours, hierarchy = cv2.findContours(edge2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contoursh, hierarchyh = cv2.findContours(edge2h,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

     
    vis2 = img.copy()
    vis2 /= 3
    vis=vis2.copy()
    vis[edge_or == 0] = (0, 255, 0)
    #vis2[edge2 == 0] = (0, 255, 0)




    
    ## array con los datos de los contornos
    borde = [[0 for x in xrange(10)] for x in xrange(10)] 
    bordeh = [[0 for x in xrange(10)] for x in xrange(10)] 
    

    ## Seleccion de contornos verticales por tamano y orientacion
    ## Se escogen los que coinciden con bordes de laser


    m=0
    for n in range(len(contours)):
    
        if len(contours[n])>4: 
            ellipse = cv2.fitEllipse(contours[n])
            (xel,yel),(em,eM),ang=ellipse
            if ang>180: ang-=180
            if em < 40 and eM >200 and ((ang<30 and ang>0) or (ang>150 and ang<180)):
                #cv2.ellipse(vis2,ellipse,VERDE,2)
                cv2.drawContours(vis2, contours, n, ROJO, 1)##representa todos
                #busca los q coincidan con los bordes detectados por el laser:
                for j in range(len(puntos_de_borde)):
                    dist=cv2.pointPolygonTest(contours[n],puntos_de_borde[j],True)
                    if dist>=-2:
                        cv2.drawContours(vis2, contours, n, VERDE_OSCURO, 1)
##                        cv2.circle(vis2,puntos_de_borde[j],9,ROJO,9)
##                        x1,y1,z1=calcula_coordenadas_3D(puntos_de_borde[j])
##                        print x1, y1, z1
                        if m<10: #lo anade a la lista de bordes buenos:
                            borde[m]=(n,xel,yel,em,eM,ang);
                            m=m+1
                        break



   ## Seleccion de contornos horizontales por tamano y orientacion
    m=0
    for n in range(len(contoursh)):
    
        if len(contoursh[n])>4: 
            ellipse = cv2.fitEllipse(contoursh[n])
            (xelh,yelh),(emh,eMh),angh=ellipse
            if angh>180: angh-=180
            if emh < 50 and eMh >100 and (angh<110 and angh>70):
                #cv2.ellipse(vis2,ellipse,VERDE,2)
##                cv2.drawContours(vis2, contoursh, n, AMARILLO, 6)
                if m<10:
                    bordeh[m]=(n,xelh,yelh,emh,eMh,angh)
                    m=m+1

    

    ## seleccion de los dos contornos verticales que encierran un hueco mayor
    borde=sorted(borde,key=itemgetter(1))
    sep_max=0
    cont_n=0
    cont_n_mas_uno=0
    
    for n in range(1,len(borde)):
       
        sep=borde[n][1]-borde[n-1][1]
        
        if sep>sep_max and borde[n][0]!=0 and borde[n-1][0]!=0: 
            sep_max=sep
            cont_n=n-1
            cont_n_mas_uno=n
            

    ## seleccion de los contornos horizontales dentro del hueco mayor            
#    for n in range(1,len(bordeh)):
#        if bordeh[n][1]< borde[cont_n_mas_uno][1] and bordeh[n][1]> borde[cont_n][1]: 
#            cv2.drawContours(vis2, contoursh, bordeh[n][0], AMARILLO, 2)
            
            
            

    cv2.drawContours(vis2, contours, borde[cont_n][0], VERDE, 1)
    cv2.drawContours(vis2, contours, borde[cont_n_mas_uno][0], VERDE, 1)
    xm1=int(borde[cont_n_mas_uno][1])
    xm2=int(borde[cont_n][1])
    ym1=int(borde[cont_n_mas_uno][2])
    ym2=int(borde[cont_n][2])
    ang1=borde[cont_n_mas_uno][5]*math.pi/180
    ang2=borde[cont_n][5]*math.pi/180
    
    if ang1>math.pi/2:
        ang1=ang1-math.pi
    if ang2>math.pi/2:
        ang2=ang2-math.pi
        





    ## Creacion de un rectangulo entre los refuerzos
##    p2 ---- p1
##    |       |
##    |       |
##    |       |
##    p3 ---- p4

        
    p1x=xm1+ym1*math.tan(ang1)
    p4x=xm1-(alto_img-ym1)*math.tan(ang1)
    p2x=xm2+ym2*math.tan(ang2)
    p3x=xm2-(alto_img-ym2)*math.tan(ang2)
    p1y=p2y=0
    p3y=p4y=alto_img
    cv2.circle(vis2,(int(p1x),int(p1y)),4,AZUL,2)
    cv2.circle(vis2,(int(p2x),int(p2y)),4,AZUL_OSCURO,2)
    cv2.circle(vis2,(int(p3x),int(p3y)),4,VERDE,2)
    cv2.circle(vis2,(int(p4x),int(p4y)),4,VERDE_OSCURO,2)
    puntos=np.array([[p1x,p1y],[p2x,p2y],[p3x,p3y],[p4x,p4y]], np.int32)

    ref1xim=(p2x+p3x)/2
    ref2xim=(p1x+p4x)/2


    #print sep_max


    ## medir tilt/pitch 

    H=alto_img
    L1=p4x-p3x
    L2=p1x-p2x

    if L1<>L2:
        y0=(L1*H)/(L1-L2)
        if y0<>0:
            y=1000/y0
            pitch=math.atan(y/1.7)
        else:
            pitch=math.pi/2
    else:
        pitch=0

        
    ##medir roll

    if xm1==ancho_img/2:
        roll=ang1
    else:
        k1=(xm2-ancho_img/2.0)/(xm1-ancho_img/2.0)
        if k1==1:
            roll=math.pi/2
        else:
            roll=(ang2-k1*ang1)/(1-k1) 



    ##medir pan/beta/yaw)

    yaw=10.806*math.pow(ang_laser,3)+2.64*ang_laser

    if pitch>0.3:
        pitch=0.3
    if pitch<-0.3:
        pitch=-0.3

    if yaw>0.3:
        yaw=0.3
    if yaw<-0.3:
        yaw=-0.3

    if roll>0.3:
        roll=0.3
    if roll<-0.3:
        roll=-0.3


    if sep_max>150:
        cv2.polylines(vis2,[puntos],True,AZUL_OSCURO,2)

    
    ## Representacion de la zona de trabajo

        margenxsup=int((p2x-p1x)/8)
        margeny=int((p1y-p3y)/12)
        margenxinf=int((p3x-p4x)/8)
    
    
        #sup der
        pb1x=int(p1x+margenxsup)
        pb1y=int(p1y-margeny)
        if pb1x<0:
            pb1x=0
        if pb1x>ancho_img-1:
            pb1x=ancho_img-1
    
        #sup izq
        pb2x=int(p2x-margenxsup)
        pb2y=int(p1y-margeny)
        if pb2x<0:
            pb2x=0
        if pb2x>ancho_img-1:
            pb2x=ancho_img-1
    
        #inf izq 
        pb3x=int(p3x-margenxinf)
        pb3y=int(p3y+margeny)
        if pb3x<0:
            pb3x=0
        if pb3x>ancho_img-1:
            pb3x=ancho_img-1
    
        #inf der 
        pb4x=int(p4x+margenxinf)
        pb4y=int(p3y+margeny)
        if pb4x<0:
            pb4x=0
        if pb4x>ancho_img-1:
            pb4x=ancho_img-1
    
        
 
    
        x1,y1,z1=calcula_coordenadas_3D(pb1x,pb1y)
        x2,y2,z2=calcula_coordenadas_3D(pb2x,pb2y)
        x3,y3,z3=calcula_coordenadas_3D(pb3x,pb3y)
        x4,y4,z4=calcula_coordenadas_3D(pb4x,pb4y)
    
    ##        print x1, y1, z1
    ##        print x2, y2, z2
    ##        print x3, y3, z3
    ##        print x4, y4, z4
            
    ##        cv2.circle(vis2,(pb1x,pb1y),4,AZUL,2)
    ##        cv2.circle(vis2,(pb2x,pb2y),4,AZUL_OSCURO,2)
    ##        cv2.circle(vis2,(pb3x,pb3y),4,VERDE,2)
    ##        cv2.circle(vis2,(pb4x,pb4y),4,VERDE_OSCURO,2)
    
        if z2>z1:
            pb2x, pb2y=calcula_coordenadas_imagen(x2, y1, z2)
            pb3x, pb3y=calcula_coordenadas_imagen(x3, y4, z3)
    
        else:
            pb1x, pb1y=calcula_coordenadas_imagen(x1, y2, z1)
            pb4x, pb4y=calcula_coordenadas_imagen(x4, y3, z4)
    
    ##        cv2.circle(vis2,(pb1x, pb1y),4,ROJO_OSCURO,2)
    ##        cv2.circle(vis2,(pb2x, pb2y),4,ROJO_OSCURO,2)
    ##        cv2.circle(vis2,(pb3x, pb3y),4,ROJO_OSCURO,2)
    ##        cv2.circle(vis2,(pb4x, pb4y),4,ROJO_OSCURO,2)
        
    
        x1,y1,z1=calcula_coordenadas_3D(pb1x,pb1y)
        x2,y2,z2=calcula_coordenadas_3D(pb2x,pb2y)
        x3,y3,z3=calcula_coordenadas_3D(pb3x,pb3y)
        x4,y4,z4=calcula_coordenadas_3D(pb4x,pb4y)

        ref1=x3 + 0.07*(x3-x4)
        ref2=x4 + 0.07*(x4-x3)
    
   
        if abs(pitch)<0.15 and abs(yaw)<0.15 and abs(roll)<0.15:

            puntos=np.array([[pb1x,pb1y],[pb2x,pb2y],[pb3x,pb3y],[pb4x,pb4y]], np.int32)
            cv2.polylines(vis2,[puntos],True,AZUL,2)
            dibuja_pinchos_en_rectangulo([pb1x,pb1y],[pb2x,pb2y],[pb3x,pb3y],[pb4x,pb4y])
            p1=(x1, y1, z1)
            p2=(x2, y1, z2)
            p3=(x2, y3, z3)
            p4=(x1, y3, z4)

            p1=(x1, y1, z1)
            p2=(x2, y2, z2)
            p3=(x3, y3, z3)
            p4=(x4, y4, z4)
            studs= calcula_puntos_en_rectangulo(p1, p2, p3, p4,roll,pitch,yaw)
            
##                    cv2.circle(vis2,(clvx,clvy),4,AZUL,2)

        else:
            studs=[[[0,0,0],[2,1,1,1]]]

        if(modo==WMODE_DETECT):
            dist_punto_m=0
            for n in range(1,len(studs)):
                dist_punto_m=dist_punto_m+studs[n][0][2]
            dist_punto_m=dist_punto_m/n
        
            dist_punto_dt=0
            for n in range(1,len(studs)):
                dist_punto_dt=dist_punto_dt+studs[n][0][2]-dist_punto_m
            dist_punto_dt=dist_punto_dt/n

            if dist_punto_m>650 and dist_punto_m<950 and abs(ref1-ref2)>400 and abs(ref1-ref2)<580:
                result=studs
            else:
                result=[[[0,0,0],[2,1,1,1]]]

        if(modo==WMODE_TRACK):
#            result=[[[0,0,0],[1,pitch,yaw,roll]]]
            if abs(ref1-ref2)>400 and abs(ref1-ref2)<580:        
                result=[[[roll,pitch,yaw],euler_a_cuaternio(roll,pitch,yaw)]]    
            else:
                result=[[[0,0,0],[2,1,1,1]]]
        if(modo==WMODE_STOPPED):
            result=[[[0,0,0],[1,1,1,1]]]
        
        
        
        return result, ref1,ref2, modo
        
       
    
    else:
        result=[[[0,0,0],[2,1,1,1]]]
        
                 
    if (representa==1):
        cv2.imshow('edge',vis2)
        cv2.imshow('laser',img_perfil_laser)
    


    return result, ref1, ref2, modo
    

def finaliza():
#    try: 
#        captura.release()
#    except:
#        pass
    cv2.destroyAllWindows()



if __name__ == "__main__":

    captura=0
    ancho_img=0
    alto_img=0
    vis2=0
    distancias_mm=0
    patron_puntos=0

    arranca()
    representa=1
    if (representa==1):
        
        cv2.namedWindow('edge')
        cv2.namedWindow('laser')
        #ch = cv2.waitKey(5)
        cv2.setMouseCallback('edge',coordenadas_punto_ventana)
        cv2.createTrackbar('th', 'laser', 190, 255, nothing)


    
    while True:
        
        ch = cv2.waitKey(5)
        calcula_dist(1,190)
        if ch == 27 or ch==1048603:
            finaliza()  
            break
    
        if ch==49 or ch==1048625:
            print calcula_dist(1,190)
            
        if ch==50 or ch==1048626:
            print calcula_dist(2,190)
            
        if ch==51 or ch==1048627:
            print calcula_dist(0,190)
            break
    
    
