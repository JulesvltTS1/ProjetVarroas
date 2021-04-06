#Ce programme prend une photo puis la traite afin d'obtenir le nombre de varroas prÃ©sent au fond de la ruche.
#Ensuite, il envoie cette information par SMS.
#Enfin, il fait tourner le moteur afin de nettoyer le tapis.


import picamera
import time
from time import sleep
import cv2
import imutils
import serial
import RPi.GPIO as GPIO # librairie pour utiliser le port GPIO.

# Setup
GPIO.setmode(GPIO.BCM) # pins par numÃ©ro sur CI.
GPIO.setwarnings(False)

#Variables :
    #Heure a laquelle le programme s'execute  :
targetTime ="14:55"
targetTime2 ="16:49"

    #Traitement :
reste = 0

    #Envoie SMS :
number ="+33********"  #NumÃ©ro de tÃ©lÃ©phone qui receptionnera les SMS.
pwrPin = 4              #Pin pour eteinddre/allumer le GSM2
GPIO.setup(pwrPin,GPIO.OUT)  # Reglage du pin en sortie

    #Rotation moteur :
# Definition des pins
M1_En = 16              #Pin autorisant l'activation
M1_In1 = 21             #Pin rotation moteur sens 1
M1_In2 = 20             #Pin rotation moteur sens 2

frequence1 = 100
rapportCyclique1 = 100

frequence2 = 100
rapportCyclique2 = 70

Pins = [M1_En, M1_In1, M1_In2]  # Creation d'une liste des pins pour chaque moteur pour compacter la suite du code

GPIO.setup(M1_En, GPIO.OUT)     # Reglage des pins en sortie
GPIO.setup(M1_In1, GPIO.OUT)
GPIO.setup(M1_In2, GPIO.OUT)

M1_Vitesse = GPIO.PWM(M1_En, frequence1)          # Reglage de la frÃ©quence et du rapport cyclique
M1_Vitesse.start(rapportCyclique1)



#Alerte Quotidienne :
def timeOfDay():
    return time.strftime("%H:%M")


#Capture & traitement image :
def capture() :
    with picamera.PiCamera() as camera:
        camera.resolution = (2592, 1944)                                              #dÃ©fini la rÃ©solution de la camÃ©ra (max :(2592, 1944); min :(64,64))
        camera.framerate = 15                                                         #dÃ©fini la frÃ©uence d'image de la camÃ©ra (dÃ©fini Ã  15 pour la rÃ©solution max)
        camera.exposure_mode = 'auto'                                                 #mode d'exposition de la camÃ©ra (par dÃ©fault : 'auto')      
        camera.start_preview()                                                        #ouverture de l'aperÃ§u de la camÃ©ra (pas nÃ©cessaire dans notre cas mais pratiquepour le tests)
        sleep(2)                                                                      #ouverture de l'objectif pendant 5s (pour rÃ©glage luminositÃ© ...)
        camera.capture('/home/pi/varroas.jpg')                                        #capture de l'image et lien de stockage
        camera.stop_preview()                                                         #fermeture de l'apercu
        print('Image CapturÃ©e')
        
def comptage() :
    image = cv2.imread('varroas.jpg')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)                                 #met l'image en noir et blanc
    flougaussien = cv2.bilateralFilter(gray, 6, 157,157)                           #floÃ»te l'image -> faire une moyenne des pixels
    edge = imutils.auto_canny(flougaussien)                                        #dÃ©termine les contours
    
    (cnts, _) = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #ajoute dans une liste les contours qu'ils trouvent suivant les mÃ©thodes de recherche
    compteur = 0                                                                   #initialisation de la variable compteur
    
    for c in cnts:
        if ( 120>cv2.contourArea(c)>85 ):                                           #calcule le pÃ©rimÃ¨tre des contours trouvÃ©s (non fermÃ©)
            compteur += 1                                                           #Incrementation du compteur de varroas
    return compteur                                                                     

    #Envoie SMS :
def envoieSMS ():
    
    message = nbVarroas - reste
    message = str(message)                                                         #convertion d'un entier en une chaine de caractere
    
    port = serial.Serial("/dev/ttyAMA0", baudrate = 115200, timeout = 3.0)          # Initialisation du port sÃ©rie reliÃ© entre la raspberry et le GSM-module M95
    print("Wait1..")
      
                                                         
    GPIO.output(pwrPin,GPIO.LOW)                                                    #Extinction de la GSM2Click
    sleep(1)

    
    port.write(b'AT+QPOWD=0\r')                                                      #Urgent power OFF
    sleep(1)
    print("Wait2...")

    GPIO.output(pwrPin,GPIO.HIGH)                                                    #Allumage de la GSM2Click 
    sleep(6)
    print ("GSM2 allume")

    port.write(b'AT+CPIN=0000\r')                                                    #Definition du code PIN. Remplacer les 0000 par le code pin de la SIM.
    sleep(1) # sleep
    print("Wait3...")

    port.write(b'AT+CMGF=1\r')  #Selection du format du message SMS. 1=Text Mode
    sleep(1)
    print("Wait4...")

    port.write(b'AT+CSCS="GSM"\r')  #Selection du format de charactere. GSM=alphabet par default 
    sleep(1)
    print("Wait4bis...")

    port.write(b'AT+CMGS="' + number.encode() + b'"\r')  #Selection du numero de telephone du destinataire
    sleep(1)
    print("Wait5...")

    port.write(b"Le systeme detecte "+message.encode()+b" varroas dans la ruche N.3. \r")  #Message a envoyer
    sleep(1)
    print("Wait6...")


    port.write(bytes([26])) #Ctrl+Z pour indiquer la fin du message en donc l'envoie
    sleep(1)
    print("Wait7...")

    result = port.read(1000)  #lecture du retour de la GSM2Click
    sleep(1)
    print(result)

    print("SMS envoyé")

#Verfication nettoyage

def verif():
    with picamera.PiCamera() as camera:
        camera.resolution = (2592, 1944)                                              #dÃ©fini la rÃ©solution de la camÃ©ra (max :(2592, 1944); min :(64,64))
        camera.framerate = 15                                                         #dÃ©fini la frÃ©uence d'image de la camÃ©ra (dÃ©fini Ã  15 pour la rÃ©solution max)
        camera.exposure_mode = 'auto'                                                 #mode d'exposition de la camÃ©ra (par dÃ©fault : 'auto')      
        camera.start_preview()                                                        #ouverture de l'aperÃ§u de la camÃ©ra (pas nÃ©cessaire dans notre cas mais pratiquepour le tests)
        sleep(2)                                                                      #ouverture de l'objectif pendant 5s (pour rÃ©glage luminositÃ© ...)
        camera.capture('/home/pi/varroasVerif.jpg')                                        #capture de l'image et lien de stockage
        camera.stop_preview()                                                         #fermeture de l'apercu
        print('Vérification effectué')
    sleep(2)
    image = cv2.imread('varroasVerif.jpg')   
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)                                 #met l'image en noir et blanc
    flougaussien = cv2.bilateralFilter(gray, 6, 157,157)                           #floÃ»te l'image -> faire une moyenne des pixels
    edge = imutils.auto_canny(flougaussien)                                        #dÃ©termine les contours    
    (cnts, _) = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #ajoute dans une liste les contours qu'ils trouvent suivant les mÃ©thodes de recherche
    compteurVerif = 0                                                                   #initialisation de la variable compteur
    
    for c in cnts:
        if ( 120>cv2.contourArea(c)>85 ):                                           #calcule le pÃ©rimÃ¨tre des contours trouvÃ©s (non fermÃ©)
            compteurVerif += 1 
    return compteurVerif


def envoieSMSerreur():
    
    port = serial.Serial("/dev/ttyAMA0", baudrate = 115200, timeout = 3.0)          # Initialisation du port sÃ©rie reliÃ© entre la raspberry et le GSM-module M95
    print("Wait1..")
      
                                                         
    GPIO.output(pwrPin,GPIO.LOW)                                                    #Extinction de la GSM2Click
    sleep(1)

    
    port.write(b'AT+QPOWD=0\r')                                                      #Urgent power OFF
    sleep(1)
    print("Wait2...")

    GPIO.output(pwrPin,GPIO.HIGH)                                                    #Allumage de la GSM2Click 
    sleep(6)
    print ("GSM2 allume")

    port.write(b'AT+CPIN=0000\r')                                                    #Definition du code PIN. Remplacer les 0000 par le code pin de la SIM.
    sleep(1) # sleep
    print("Wait3...")

    port.write(b'AT+CMGF=1\r')  #Selection du format du message SMS. 1=Text Mode
    sleep(1)
    print("Wait4...")

    port.write(b'AT+CSCS="GSM"\r')  #Selection du format de charactere. GSM=alphabet par default 
    sleep(1)
    print("Wait4bis...")

    port.write(b'AT+CMGS="' + number.encode() + b'"\r')  #Selection du numero de telephone du destinataire
    sleep(1)
    print("Wait5...")

    port.write(b"Erreur survenue lors du nettoyage. \r")  #Message a envoyer
    sleep(1)
    print("Wait6...")


    port.write(bytes([26])) #Ctrl+Z pour indiquer la fin du message en donc l'envoie
    sleep(1)
    print("Wait7...")

    result = port.read(1000)  #lecture du retour de la GSM2Click
    sleep(1)
    print(result)

    print("SMS envoyé")
#Rotation moteur :

def sens1() :
    GPIO.output(Pins[1], GPIO.HIGH)
    GPIO.output(Pins[2], GPIO.LOW)
    print("Le moteur tourne dans le sens 1.")

def sens2() :
    GPIO.output(Pins[1], GPIO.LOW)
    GPIO.output(Pins[2], GPIO.HIGH)
    print("Le moteur tourne dans le sens 2.")

def arret() :
    GPIO.output(Pins[1], GPIO.LOW)
    GPIO.output(Pins[2], GPIO.LOW)
    print("Le moteur est Ã  l'arret.")

def rouleau():
    sens1()
    sleep(16)
    arret()
    sleep(1)
    
def rouleauErreur():
    M1_Vitesse.ChangeFrequency(frequence2)
    M1_Vitesse.ChangeDutyCycle(rapportCyclique2)
    sens2()
    sleep(7)
    arret()
    sleep(1)
    
while True:
    while timeOfDay() != targetTime and timeOfDay() != targetTime2:  #si nous ne sommes pas a l'heure voulu, on attend la bonne heure pour executer les fonctions
        sleep(5)
    capture()
    sleep(5)
    nbVarroas = comptage()
    sleep(5)
    rouleau()
    sleep(5)
    reste = verif()
    if nbVarroas == reste and reste != 0 or 1 :
        envoieSMSerreur()
        rouleauErreur()
    
    while timeOfDay() == targetTime or timeOfDay() == targetTime2:  #si nous sommes a l'heure voulu on temporise le temps que les fonctions s'executent
        sleep(5)