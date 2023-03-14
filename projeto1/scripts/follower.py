#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#   exemplo adaptado do livro:
#   
#  Programming Robots with ROS.
#  A Practical Introduction to the Robot Operating System
#  Example 12-5. follower_p.py pag265
#  
#  Referendia PD:https://github.com/martinohanlon/RobotPID/blob/master/mock/mock_robot_pd.py 
import rospy
import numpy as np
import math
import cv2
import time
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
from nav_msgs.msg import Odometry
import helper
import cv2.aruco as aruco
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64




class Follower:

    def __init__(self):
        
        #Para poder seguir o creeper desejado dois inputs foram colocados para selecionar a cor e o id
        self.cor_creeper = input("Qual cor do creeper?")
        self.id_creeper = input("digite o id do creeper: ")
        
        self.bridge = CvBridge()
        self.cv_image = None
        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters  = aruco.DetectorParameters_create()
        self.camera_matrix   = np.loadtxt('cameraMatrix_raspi.txt', delimiter=',')
        self.camera_distortion   = np.loadtxt('cameraDistortion_raspi.txt', delimiter=',')
        self.marker_size  = 20 #- [cm]

	    #topico da camera do robo real
	    #self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed',
        
        #robo simulacao
        #'/camera/image/compressed'

        #subscribers e publishers do robo
        self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed', 
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                             Twist, 
                                             queue_size=1)
        
        self.laser_subscriber = rospy.Subscriber('/scan',
                                                  LaserScan, 
	 		                                    self.laser_callback)

        self.odom_subscriber = rospy.Subscriber('/odom',
                                                  Odometry, 
	 		                                    self.odom_callback)

        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)
        
        self.twist = Twist()
        self.laser_msg = LaserScan()
         
        
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1
        self.angulo = 0


        #Variaveis de estado e counters utilizados utilizados nas funcoes

        #Variaveis de obstaculo relacionadas ao lidar para fazer o slalom
        self.OBSTACULO = False 
        self.OBSTACULO_FRENTE_DIREITA = False
        self.OBSTACULO_FRENTE_ESQUERDA = False
        self.OBSTACULO_DIREITA = False
        self.OBSTACULO_ESQUERDA = False
        self.OBSTACULO_DIREITA_TOTAL = False
        self.OBSTACULO_ESQUERDA_TOTAL = False

        #Variaveis do creeper relacionadas ao lidar para garantir o funcionamento correto da garra
        self.OBSTACULO_CREEPER_DIREITA = False
        self.OBSTACULO_CREEPER_ESQUERDA = False

        self.SLALOM_FEITO = True #Variavel de estado para o slalom
        self.ve_linha = False #Variavel para garantir que o robo esta vendo a linha
        self.viuvermelho1 = False #Variavel utilizada para garantir que o robo passou da primeira caixa do slalom 
        self.distancia_origem = 0 #Variavel utilizada na odometria para medir a distancia do lugar de origem
        self.partiu_da_origem = False #Variavel utilizada para checar se o robo iniciou o circuito
        self.posicao_x = None #Variavel utilizada para odometria (posicao relacionada ao eixo x)
        self.posicao_y = None #Variavel utilizada para odometria (posicao relacionada ao eixo y)
        self.x0 = None #Variavel utilizada para odometria (posicao inicial relacionada ao eixo x)
        self.y0 = None #Variavel utilizada para odometria (posicao inicial relacionada ao eixo y)
        self.esta_perto = False #Variavel utilizada para odometria (para checar se o robo esta perto da posicao inicial)
        self.conta_perto = 0 #Contador utilizado para odometria (para checar quantas vezes o robo passou pela posicao inicial)
        self.viu_id = False #Variavel utilizada para checar se o robo viu o algum id do aruco
        self.ve_creeper= False #Variavel utilizada para checar se o robo ve o contorno do creeper
        self.mata_creeper = False #Variavel para fazer com que o robo entre na funcao de derrubar o creeper
        self.ids = "" #Variavel utilizada para guardar o id que esta sendo visto
        self.conta_garra = 0 #Variavel utilizada para garanti que a garra seja utilizada so uma vez

        


#     #     self.lastError = 0
#     #     self.max_vel_linear = 0.2
#     #     self.max_vel_angular = 2.0
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)
    
    def odom_callback(self, msg):

        """Funcao utilizada para fazer a odometria"""

        quat = msg.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]

        #pegando as posicoes do robo
        posicao = msg.pose.pose.position
        self.posicao_x = posicao.x
        self.posicao_y = posicao.y

        #pegando as posicoes iniciais do robo
        if self.x0 is None:
            self.x0 = self.posicao_x
        if self.y0 is None:
            self.y0 = self.posicao_y
        
        #Utilizando distancia euclidiana para checar a distancia que o robo esta da origem
        self.distancia_origem = math.sqrt((self.posicao_x - self.x0)**2 + (self.posicao_y - self.y0)**2)

        if self.partiu_da_origem is False:
            self.partiu_da_origem = True
        
        #print(f"a distancia para origem e:{self.distancia_origem}")

        angulos = np.degrees(transformations.euler_from_quaternion(lista))
        self.angulo = angulos[2] # Eixo Z
        #print(f"angulo:{self.angulo}")


    def laser_callback(self, msg):

        """Funcao utilizada para receber as informacoes do lidar"""

        #print(f"Distancia: {self.laser_msg.ranges[0]}")
        
        self.obstaculo_frente_direita_l = []
        self.obstaculo_frente_esquerda_l = []
        self.obstaculo_direita_l = []
        self.obstaculo_esquerda_l = []
        self.obstaculo_direita_total_l = []
        self.obstaculo_esquerda_total_l = []

        """Devido ao fato de que, ao utilizar o lidar do robo real, 
        o lidar retornava zero quando nao havia obstaculo, 
        todos os valores zero recebidos do lidar foram transformados em 10 """

        for i in enumerate(msg.ranges[330:359]):
            if i[1] != 0:
                self.obstaculo_frente_direita_l.append(i[1])
            else:
                self.obstaculo_frente_direita_l.append(10)

        for i in enumerate(msg.ranges[0:30]):
            if i[1] != 0:
                self.obstaculo_frente_esquerda_l.append(i[1])
            else:
                self.obstaculo_frente_esquerda_l.append(10)
        
        for i in enumerate(msg.ranges[300:329]):
            if i[1] != 0:
                self.obstaculo_direita_l.append(i[1])
            else:
                self.obstaculo_direita_l.append(10)

        for i in enumerate(msg.ranges[31:60]):
            if i[1] != 0:
                self.obstaculo_esquerda_l.append(i[1])
            else:
                self.obstaculo_esquerda_l.append(10)
        
        for i in enumerate(msg.ranges[270:300]):
            if i[1] != 0:
                self.obstaculo_direita_total_l.append(i[1])
            else:
                self.obstaculo_direita_total_l.append(10)
        
        for i in enumerate(msg.ranges[61:90]):
            if i[1] != 0:
                self.obstaculo_esquerda_total_l.append(i[1])
            else:
                self.obstaculo_esquerda_total_l.append(10)

        if min(self.obstaculo_frente_direita_l) < 0.5:
            self.OBSTACULO_FRENTE_DIREITA = True
            self.OBSTACULO = True
           # print("Obstaculo frente direita")
        
        else:
            self.OBSTACULO_FRENTE_DIREITA = False


        if min(self.obstaculo_frente_esquerda_l) < 0.5: #se existe algum obsatculo a menos de 30cm em um range de 45 graus
            self.OBSTACULO_FRENTE_ESQUERDA = True #tem um obstaculo
            self.OBSTACULO = True
          #  print("Obstaculo frente direita")

        else: 
            self.OBSTACULO_FRENTE_ESQUERDA = False
        

        if min(self.obstaculo_esquerda_l) < 0.5: 
            self.OBSTACULO_ESQUERDA = True
            self.OBSTACULO = True
          #  print("Obstaculo esquerda")
        
        else:
            self.OBSTACULO_ESQUERDA = False


        if min(self.obstaculo_direita_l) < 0.5: 
            self.OBSTACULO_DIREITA = True
            self.OBSTACULO = True
           # print("Obstaculo direita")    
        
        else:
            self.OBSTACULO_DIREITA = False
        
        if min(self.obstaculo_direita_total_l) < 0.5:
            self.OBSTACULO_DIREITA_TOTAL = True
            self.OBSTACULO = True
          #  print("Obstaculo direita total")
        
        else:
            self.OBSTACULO_DIREITA_TOTAL = False


        if min(self.obstaculo_esquerda_total_l) < 0.5:
            self.OBSTACULO_ESQUERDA_TOTAL = True
            self.OBSTACULO = True
          #  print("Obstaculo esquerda total")
        
        else:
            self.OBSTACULO_ESQUERDA_TOTAL = False

        if self.OBSTACULO_FRENTE_DIREITA is True or self.OBSTACULO_FRENTE_ESQUERDA is True or self.OBSTACULO_DIREITA is True or self.OBSTACULO_ESQUERDA is True or self.OBSTACULO_DIREITA_TOTAL is True or self.OBSTACULO_ESQUERDA_TOTAL is True:
            self.OBSTACULO = True
        else:
            self.OBSTACULO = False
        
        #Para checar se o creeper esta a menos de 19 cm do robo
        if min(self.obstaculo_frente_direita_l) < 0.19:
            self.OBSTACULO_CREEPER_DIREITA = True
            print("creeper direita")
        
        else:
            self.OBSTACULO_CREEPER_DIREITA = False
        
        if min(self.obstaculo_frente_esquerda_l) < 0.19:
            self.OBSTACULO_CREEPER_ESQUERDA = True
            print("creeper esuqerda")
        
        else:
            self.OBSTACULO_CREEPER_ESQUERDA = False

    
    def image_callback(self, msg):

        """Funcao para receber os dados recebidos da camera"""
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
	        #cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real
            aruco_image = cv_image.copy()
            self.aruco(aruco_image)

        except CvBridgeError as e:
            print('ex', e)


        
        #mascara para o amarelo
        cv2.imshow("window", cv_image)
        cv2.waitKey(1)

        #SEGMENTANDO A LINHA 
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #cores vida real
        lower_yellow = np.array([ 17,  31, 200],dtype=np.uint8)
        upper_yellow = np.array([45, 205, 255],dtype=np.uint8)

        lower_yellow2 = np.array([ 20,  8, 97],dtype=np.uint8)
        upper_yellow2 = np.array([45, 94, 211],dtype=np.uint8)

        #cores simulacao
        #lower_yellow = np.array([ 20,  48, 157],dtype=np.uint8)
        #upper_yellow = np.array([32, 255, 255],dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))

        #IMAGENS UTILIZADAS PARA SEREM CORTADAS AO LER O ARUCO
        img_direita = hsv.copy()
        mask_direita = cv2.inRange(img_direita, lower_yellow, upper_yellow)
        mask_direita = cv2.morphologyEx(mask_direita, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
  

        img_esquerda = hsv.copy()
        mask_esquerda = cv2.inRange(img_esquerda, lower_yellow2, upper_yellow2)
        mask_esquerda = cv2.morphologyEx(mask_esquerda, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))

        #cortando a imagem
        h, w, d = cv_image.shape
        search_top1 = 3*h//4
        #search_bot1 = 3*h//4 + 20
        mask[0:search_top1, 0:w] = 0
        #mask[search_bot1:h, 0:w] = 0

        mask_esquerda[0:search_top1-300, 330:w] = 0
        mask_direita[0:search_top1 - 300, 0:500] = 0

        #SEGMENTANDO OS CREEPERS
        if self.cor_creeper == "red":
            baixo = np.array([1,176,112])
            alto  = np.array([5,255,255])
             
        
        if self.cor_creeper == "blue":
            baixo = np.array([86, 172, 36])
            alto  = np.array([116, 255, 130])
        
        if self.cor_creeper == "green":
            baixo = np.array([44, 132, 31])
            alto  = np.array([87, 255, 137])     

        mask_creepers = cv2.inRange(hsv, baixo, alto)  

        self.w = w
        self.h = h
        M = cv2.moments(mask)
        D = cv2.moments(mask_direita)
        E = cv2.moments(mask_esquerda)
        C = cv2.moments(mask_creepers)

        ARUCOS = [200, 150, 21, 11, 13, 27, 51, 100]

        #SELECIONANDO OS CENTROS DE MASSA DE ACORDO COM O QUE O ROBO DEVE FAZER
        if self.ids.all() not  in ARUCOS:
             if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)
            
        try:
            if (self.ids == 200 or self.ids == 100) and self.distance < 300:
                self.SLALOM_FEITO = True
            # print("virando a direita")
                if D['m00'] > 0:
                    self.cx = int(D['m10']/D['m00'])
                    self.cy = int(D['m01']/D['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)
            if self.ids == 150 and self.distance < 300:
                print("virando a esquerda")
                if E['m00'] > 0:
                    self.cx = int(E['m10']/E['m00'])
                    self.cy = int(E['m01']/E['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)

            if (int(self.ids[0]) == int(self.id_creeper)) :
                self.viu_id = True
            else:
                self.viu_id = False
                    


        except:
            pass

        maior_contorno_yellow = helper.encontrar_maior_contorno(mask)
        maior_contorno_yellow_esquerda = helper.encontrar_maior_contorno(mask_esquerda)
        maior_contorno_yellow_direita = helper.encontrar_maior_contorno(mask_direita)
        maior_contorno_creeper = helper.encontrar_maior_contorno(mask_creepers)

        if maior_contorno_yellow is not None:
            self.ve_linha = True
        if maior_contorno_yellow is None or cv2.contourArea(maior_contorno_yellow) < 100:
            self.ve_linha = False

        if maior_contorno_creeper is not None :
            self.ve_creeper= True

        if maior_contorno_creeper is None or cv2.contourArea(maior_contorno_creeper) < 40:
            self.ve_creeper = False

        #IDENTIFICANDO O CENTRO DE MASSA DO CONTORNO DO CREEPER
        if self.ve_creeper is True and self.viu_id is True:
            self.mata_creeper = True

            if  C['m00'] > 0:                
                    self.cx = int(C['m10']/C['m00'])
                    self.cy = int(C['m01']/C['m00'])
                    cv2.circle(cv_image, (self.cx, self.cy), 20, (0,0,255), -1)
        else:
            self.mata_creeper = False


        

        cv2.imshow("window", cv_image)
        cv2.imshow("Referencia", mask)
        cv2.imshow("direita", mask_direita)
        cv2.imshow("esquerda", mask_esquerda)
        cv2.imshow("creepers", mask_creepers)
        cv2.waitKey(1)
        
    
    
    def control_1(self):

        """Controle utilizado quando o robo deve somente seguir a linha"""

        ### BEGIN CONTROL
        err = self.cx - self.w/2
        #------controle P simples--------------------

        self.twist.linear.x = 0.12

        self.twist.angular.z = -float(err) / 3500

    def procura_linha(self):

        """Funcao utilizada para quando o robo perde a linha"""

        self.twist.linear.x = 0.0
        self.twist.angular.z = -0.1




    def gira_90(self):

        """Funcao para o robo realizar o slalom"""

        if self.OBSTACULO_FRENTE_DIREITA is True or self.OBSTACULO_FRENTE_ESQUERDA is True and self.OBSTACULO_DIREITA is True and self.OBSTACULO_DIREITA_TOTAL is False:
            print("gira primeiro")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.1

        if self.OBSTACULO_DIREITA_TOTAL is True and self.OBSTACULO_FRENTE_DIREITA is False:
            print("desvia primeiro")
            self.twist.linear.x = 0.08
            self.twist.angular.z = -0.15

        if self.OBSTACULO_FRENTE_DIREITA is True and self.OBSTACULO_DIREITA_TOTAL is True and (self.OBSTACULO_ESQUERDA is True or self.OBSTACULO_FRENTE_ESQUERDA is True):
            print("gira segunda")
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.1

        if self.OBSTACULO_ESQUERDA_TOTAL is True and self.OBSTACULO_FRENTE_ESQUERDA is False:
            print("desvia segunda")
            self.twist.linear.x = 0.07
            self.twist.angular.z = 0.12

            self.viuvermelho1 = True

        if self.OBSTACULO_DIREITA_TOTAL is True and self.OBSTACULO_FRENTE_DIREITA is False and self.OBSTACULO_FRENTE_ESQUERDA is False and self.OBSTACULO_ESQUERDA_TOTAL is False and self.OBSTACULO_DIREITA is True and self.viuvermelho1 is True:
            print("desvia terceira")
            self.twist.linear.x = 0.08
            self.twist.angular.z = -0.08

            #self.viuvermelho1 = False


        
    
    def control_2(self):

        """Controle que "gere" o robo e seleciona todas as decisoes a serem tomadas de acordo com as variaveis de estado"""

        if self.OBSTACULO is True and self.SLALOM_FEITO is False:
            self.gira_90()
            
        
        elif self.ve_linha is True and self.mata_creeper is False:
            self.control_1()
            
        elif self.ve_linha is False  and self.mata_creeper is False:
            self.procura_linha()
            #print("Ta virando")

        if self.distancia_origem < 1.5 and self.partiu_da_origem is True and self.SLALOM_FEITO is True and self.esta_perto is False:
            self.conta_perto += 1
            self.esta_perto = True
            
            #print(self.conta_perto)
        
        if self.conta_perto == 2:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0


        if self.mata_creeper is True:
            #print("matar")
            self.creeper()
 
        #print(self.conta_perto)
    
        ### END CONTROL
        #publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        #rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
        self.rate.sleep()

    def aruco(self, cv_image):

        """Funcao para o aruco"""
        
        #cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8") 
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        corners, self.ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        #print(self.ids)

        self.ids = np.array(self.ids).flatten()

        #print(self.ids)

        if self.ids is not None:
            #-- ret = [rvec, tvec, ?]
            #-- rvec = [[rvec_1], [rvec_2], ...] vetor de rotação
            #-- tvec = [[tvec_1], [tvec_2], ...] vetor de translação

            try:
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                
                #####################---- Distancia Euclidiana ----#####################
                # Calcula a distancia usando apenas a matriz tvec, matriz de tanslação
                # Pode usar qualquer uma das duas formas

                self.distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
                #print("Distancia Euclidiana: ", self.distance)
                #print("ID: ", self.ids)

                distance = np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
                distancenp = np.linalg.norm(tvec)
                #print("Distancia Euclidiana: ", distance)
            except:
                pass

    def creeper (self):

        """ Funcao para controlar o robo no momento em que ele esta buscando o creper"""
        ### BEGIN CONTROL
        err = self.cx - self.w/2
        #------controle P simples--------------------

        self.twist.linear.x = 0.07
        self.twist.angular.z = -float(err) / 4000
        
        if self.conta_garra == 0:
            if self.OBSTACULO_CREEPER_DIREITA is True and self.OBSTACULO_CREEPER_ESQUERDA is False:
                print("levanta")
                self.twist.linear.x = 0.0
                self.twist.angular.z =-0.08
                self.ombro.publish(1.0)
                self.conta_garra =1 
            elif self.OBSTACULO_CREEPER_ESQUERDA is True and self.OBSTACULO_CREEPER_DIREITA is False:
                print("levanta")
                self.twist.linear.x = 0.0
                self.twist.angular.z =0.08
                self.ombro.publish(1.0)
                self.conta_garra =1
            elif self.OBSTACULO_CREEPER_ESQUERDA is True and self.OBSTACULO_CREEPER_DIREITA is True:
                print("levanta")
                self.twist.linear.x = 0.01
                self.twist.angular.z =0.0
                self.ombro.publish(1.0)
                self.conta_garra =1  
        elif self.conta_garra == 1:
            self.ombro.publish(-1.0)
            self.conta_garra = 2

        

# Main loop
if __name__=="__main__":
    rospy.init_node('follower')
    follower = Follower()
    while not rospy.is_shutdown():
        follower.control_2()