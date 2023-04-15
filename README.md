﻿# Robot Pioneer - Modelo de Control de Seguimiento de Puntos
El Robot Pioneer es un robot diferencial con capacidad de navegación autónoma en entornos desconocidos. En este proyecto, se desarrolló un modelo de ecuaciones para un control de seguimiento de puntos utilizando MATLAB y Coppelia.

Modelo de Ecuaciones
El control de seguimiento de puntos implica el movimiento del robot Pioneer para seguir una trayectoria especificada, que consiste en una secuencia de puntos en un plano 2D. El modelo de ecuaciones para el control de seguimiento de puntos en el Robot Pioneer se describe mediante las siguientes ecuaciones de cinemática:

Cálculo de la distancia entre el robot Pioneer y el punto objetivo:
matlab
Copy code
d = sqrt((Xp-Xt)^2 + (Yp-Yt)^2)
Donde:

d es la distancia entre el robot y el punto objetivo

Xp y Yp son las coordenadas de la posición actual del robot Pioneer

Xt y Yt son las coordenadas del punto objetivo

Cálculo de la velocidad lineal deseada del robot:

matlab
Copy code
v = kpt * d
Donde:

v es la velocidad lineal deseada del robot

kpt es la constante de translación

Cálculo del ángulo deseado hacia el punto objetivo:

matlab
Copy code
θd = atan2(Yt - Yp, Xt - Xp)
Donde:

θd es el ángulo deseado hacia el punto objetivo

Xp y Yp son las coordenadas de la posición actual del robot Pioneer

Xt y Yt son las coordenadas del punto objetivo

Cálculo de la velocidad angular del robot:

matlab
Copy code
ω = -kpr * (θ - θd)
Donde:

ω es la velocidad angular del robot

kpr es la constante de rotación

θ es el ángulo actual del robot con respecto al eje X

θd es el ángulo deseado hacia el punto objetivo

Cálculo de las velocidades individuales de las llantas del robot Pioneer:

matlab
Copy code
Vr = v + (L * ω) / 2;
Vl = v - (L * ω) / 2;
Donde:

Vr y Vl son las velocidades lineales de las llantas derecha e izquierda, respectivamente
v es la velocidad lineal deseada del robot
L es la distancia entre las llantas del robot
ω es la velocidad angular del robot
Implementación en MATLAB y Coppelia
La implementación se lleva a cabo utilizando MATLAB y Coppelia. En MATLAB, se utilizan las API y las librerías disponibles para la comunicación con Coppelia. Se declaran los objetos necesarios, se calcula la posición del robot Pioneer, se obtienen los valores de las ecuaciones de cinemática, se calculan las velocidades individuales de las llantas y se envían a Coppelia para que el robot responda en consecuencia.

Para ejecutar el código, se debe tener instalado MATLAB y Coppelia, y se deben seguir las
