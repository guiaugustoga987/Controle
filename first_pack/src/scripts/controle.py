#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

Laser_msg = None
ranges = None
xr=None
yr=None
thr=None

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass
 
def callback_odom(msg):
    #Função que retorna a Odometria do Drone. A função recebe a msg de posição do drone e guarda nas variáveis [xr,yr,thr] os valores respectivos de x, y e ângulo do robô.
    global xr
    global yr
    global thr
    global roll
    global pitch
    global yaw
    global tf
    
    xr=msg.pose.pose.position.x
    yr=msg.pose.pose.position.y
    thr = np.arctan2(2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z,1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z) 
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]


       
def callback_laser(msg): 

    #Recepção de variáveis globais.
    global xg
    global yg
    global thg
    global JaCalculouObjetivo
    global e0
    global distancia_obstaculos
    global Angulos_obstaculos
    global angulo_atual
    global distobs
    global somatorio
    global soma
    global ranges
    global Laser_msg
    
    
    

    Laser_msg=msg
    ranges = np.array(Laser_msg.ranges)
    Angulos_obstaculos = np.array(ranges)

  #  e0 = 5

    #Se o objetivo não foi calculado:
    if JaCalculouObjetivo == 0:
       JaCalculouObjetivo = 1 #Atualização que o objetivo já foi lido.
      

       
        #Descobrir o limites do sensores

       min_value = 361
       max_value = -361
       max_range = 0
       min_range = 0
       Angulos_obstaculos = 0
       x0 = 0
       y0 = 0



       Angulos_Porta = np.where(ranges > 10)
       
       

       min_value = np.min(Angulos_Porta)
       max_value = np.max(Angulos_Porta)
       max_range = ranges[max_value+1]
       min_range = ranges[min_value-1]

       min_value = min_value - 180
       max_value = max_value - 179

       #print('Angulo Minimo:',min_value)
       #print('Angulo Máximo:',max_value)

       # Goal
       #Distancia Médio da Porta
       range_medio = ((float(max_range) + float(min_range))/2)
       thg = ((float(max_value) + float(min_value))/2)*np.pi/180.
       #xg= ((range_medio)*np.cos(thg)) -0.75
       #yg= ((range_medio)*np.sin(thg)) - 0.85
       xg = 1.05
       yg = 3.26


       '''
       angulo_atual=[]
       distobs=[]
       somatorio =[]


       for i in range(len(Angulos_obstaculos)):
           if(Angulos_obstaculos[i] < e0):
               angulo_atual.append(i-180)
               distobs.append(Angulos_obstaculos[i])  #distobs[i] = angulos_obstaculos [i] : Distância entre o robô e o obstáculo i

        
        
       for j in range(len(angulo_atual)):
           x0 = (xr+distobs[j]*np.cos(angulo_atual[j])) #Cálculo de x0
           y0 = (yr+distobs[j]*np.sin(angulo_atual[j])) #Cálculo de y0
           dx0 = xr - x0
           dy0 = yr - y0
           somatorio.append((1/(distobs[j]**3))*((1/distobs[j])-(1/e0))*np.array([[dx0],[dy0]]))
           
              
              
       angulo_atual = np.array(angulo_atual)
       distobs = np.array(distobs)
       somatorio = np.array(somatorio)
       soma = sum(somatorio)
       
       #print('xg',xg)
       #print('yg',yg)
       
   
       #print('Angulos',angulo_atual)
      # print('Distancias',distobs)
      # print('Soma',soma)
       #print('Somatorio',somatorio)
       
       '''
       '''
        Laser_msg=msg
        ranges = np.array(Laser_msg.ranges) #Descobrir o limites do sensores
       # distancia_obstaculos = np.where(ranges < 10)
        Angulos_obstaculos = np.array(ranges)

        angulo_atual=[]
        distobs=[]
        somatorio=[]
        
        
        for i in range(len(Angulos_obstaculos)):
           if(Angulos_obstaculos[i] < e0):
               angulo_atual.append(i-180)
               distobs.append(Angulos_obstaculos[i])

               
                        

        for j in range(len(angulo_atual)):
           x0 = (xr+distobs[j]*np.cos(angulo_atual[j])) #Cálculo de x0
           y0 = (yr+distobs[j]*np.sin(angulo_atual[j])) #Cálculo de y0
           dx0 = xr - x0
           dy0 = yr - y0
           somatorio.append((1/(distobs[j]**3))*((1/distobs[j])-(1/e0))*np.array([[dx0],[dy0]]))
        
        somatorio = np.array(somatorio)
        soma = sum(somatorio)
        distobs = np.array(distobs)
        angulo_atual = np.array(angulo_atual)
        
        
        #print('xg',xg)
        #print('yg',yg)
        return soma
        #print('Angulos',angulo_atual)
        #print('Distancias',distobs)  
        # ''' 
  
        
    return Angulos_obstaculos
 



def aulagazebo():

    #Declaração de Variáveis globais.
    global xg
    global yg
    global thg
    global JaCalculouObjetivo
    global xr
    global yr
    global thr
    global thr1
    global distobs
    global e0
    global distancia_obstaculos
    global Angulos_obstaculos
    global angulo_atual
    global somatorio
    global soma
    global var
    
    


    #Inicializando os valores das variaveis
    xg = 3.53 #Objetivo em x do robô
    yg = 10 #Objetivo em y do robô
    thg = 0.0 #Objetivo em ângulo do robõ

    
    x0 = 0.0
    y0 = 0.0

    #Variavel se já ocorreu a leitura e calculo do objetivo.
    JaCalculouObjetivo = 0

    #Posição real do Drone.
    xr = 0.0
    yr = 0.0
    thr = 0.0
    e0 = 2
    soma = np.array([[0],[0]])
    fatt = np.array([[0],[0]])
    frep = np.array([[0],[0]])
    ftot = np.array([[0],[0]])
    gama1 = 0
    alpha1 = 0
    thr = 0
    thr1 = 0

    ###### SETUPPP #########
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1) #Cria pub_vel, publicando no tópico /cmd_vel com formato Twist.

    rospy.init_node('aulagazebo', anonymous=False) #Inicializa o Nó "aulagazebo"

    rospy.Subscriber('/scan', LaserScan, callback_laser)  #Chama a função callback_laser quando recebe uma informação do tópico /scan
    rospy.Subscriber('/odom',Odometry,callback_odom)  #Chama a função callback_odom quando recebe uma informação do tópico /scan
    rate = rospy.Rate(10) # 10hz
    #Laser_msg=msg
    #ranges = np.array(Laser_msg.ranges) #Descobrir o limites do sensores


    # Velocity Message    
    twist = Twist()
    twist.linear.x = 0 
    twist.linear.y = 0 
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 
    twist.angular.z = 0
    rospy.sleep(2) 
    
    ######## LOOOP  ########
    while not rospy.is_shutdown():  #Enquanto o programa não é fechado!
     
	# Ganhos do Controlador de posição. kp é o ganho em relação a distância ao alvo. ka é o ganho referente a diferença entre o angulo para chegar ao objetivo e o do robô (alfa) e kb é do angulo do objetivo e o angulo do robô
        
        ka = 0.15 # 0.05 
        #katt = 15 #com 2 obstaculos
        #krep = 20 # com 2 obstaculos
        katt = 20 #com 2 obstaculos
        krep = 30 # com 2 obstaculos

        
        
        #Erro entre a posição e angulos atuais do robô e o objetivo
        
        dx = xr - xg #Diferença entre a posição objetivo do drone e a atual no eixo x.
        dy = yr - yg #Diferença entre a posição objetivo do drone e a atual no eixo x.
        dx0 = xr - x0 #Diferença entre a posição do robô e o obstáculo no eixo x
        dy0 = yr - y0 # Diferença entre a posição do robô e o obstáculo no eixo y
        rho = np.sqrt(dx**2+dy**2) #Distância entre o objetivo e a posição atual do drone.
        
            
              
           

        #Enquanto o Erro for maior que um limiar, Calcule a velocidade linear e Angular do robô. Caso o erro seja menor, atualize para zero para parar o robô.
        if rho > 0.1:

           # if thr < 0:
               # thr = 6.28 - abs(thr)
               
                
            
            # Controle

            # distancia_obstaculos = np.where(ranges < 10)
            #Angulos_obstaculos = np.array(ranges)
            angulo_atual=[]
            distobs=[]
            somatorio=[]
            x00 = []
            y00= []
            cosseno = []


            for i in range(len(Angulos_obstaculos)):
                if(Angulos_obstaculos[i] < e0):
                    angulo_atual.append(i)
                    distobs.append(Angulos_obstaculos[i])

            angulo_atual = np.array(angulo_atual)
            distobs = np.array(distobs)
            if len(angulo_atual) > 0 :

                for j in range(len(angulo_atual)):
                    x0 = xr+(distobs[j]*(np.cos(angulo_atual[j]*np.pi/180))) #Cálculo de x0
                    y0 = yr+(distobs[j]*(np.sin(angulo_atual[j]*np.pi/180))) #Cálculo de y0
                    x00.append(xr+(distobs[j]*(np.cos(angulo_atual[j]*np.pi/180))))
                    y00.append(yr+(distobs[j]*(np.sin(angulo_atual[j]*np.pi/180))))
                    dx0 = xr - x0
                    dy0 = yr - y0
                    somatorio.append((1/(distobs[j]**3))*((1/distobs[j])-(1/e0))*np.array([[dx0],[dy0]]))
                    cosseno.append(np.cos(angulo_atual[j]*np.pi/180))
                

                somatorio = np.array(somatorio)
                soma = sum(somatorio)
                x00 = np.array(x00)
                y00 = np.array(y00)
            else:
                somatorio = np.array([[0],[0]])
                soma = np.array([[0],[0]])
                    
        

            var = len(angulo_atual)

            
            

            
            dx = xr - xg #Diferença entre a posição objetivo do drone e a atual no eixo x.
            dy = yr - yg #Diferença entre a posição objetivo do drone e a atual no eixo x.
            rho = np.sqrt(dx**2+dy**2) #Distância entre o objetivo e a posição atual do drone.
            fatt = -katt*np.array([[dx],[dy]])
            
            if var == 0:
                frep = np.array ([[0],[0]])
            else:
                frep = krep*soma
    

            #print('somatorio',xr) 
               

            
            ftot = fatt + frep
            gama1 = np.arctan2(ftot[1],ftot[0]) 
            alpha1 = gama1-thr
         #   gamadeg = gama1*180/np.pi
          #  thrdeg = thr*180/np.pi
          #  alphadeg = gamadeg - thrdeg 

        
            
           # print('var',var)
            #print('yr',yr)
           # print('x0',x00[30])
           # print('y0',y00[30])
           # print('distobs',distobs[30])
           # print('angulo',angulo_atual)
           # print('angulocru',Angulos_obstaculos[30])
            print('xr',xr)
            print('yr',yr)
            print('xg',xg)
            print('yg',yg)
            #print('cosseno',cosseno[30])
            #print('alpha1',alpha1)
            #print('gama1',gama1)
           # print('thr',thr)
            #print('alphadeg',alphadeg)
            #print('gamadeg',gamadeg)
           # print('thrdeg',thrdeg)
            
            #print('ftot',ftot)
            #print('angulo_atual',angulo_atual)



                   
            
            #gama = np.arctan2(dy,dx) # #Angulo entre o objetivo e 0º.
            #alpha = gama  #Angulo entre a frente do drone e o objetivo
            #beta = thg - gama #Diferença entre o objetivo e a o vetor rho

            #Atualize a velocidade para zero.
            v=0
            w=0

            #Controlador proporcional de velocidade linear
            v=0.2

            if rho < 2:
                v = 0.1

            #Controlador proporcional de velocidade Angular.
            w = ka * alpha1
            print('rho',rho)
            #angulo_atual=[0]
            

        else:
            #Chegou no Objetivo, Pare a velocidade
            v=0
            w=0

        #Atualize a mensagem twist com os valores de velocidade linear e angular calculados.
        twist.linear.x = v
        twist.angular.z = -w
        #Publique essa mensagem no tópico.
        pub_vel.publish(twist)
        w = 0.0
        frep = 0.0
        fatt = 0.0
        gama1 = 0.0
        rate.sleep()

if __name__ == '__main__':
    try:
        aulagazebo()
    except rospy.ROSInterruptException:
        pass
