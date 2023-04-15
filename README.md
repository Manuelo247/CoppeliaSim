# Robot Pioneer - Modelo de Control de Seguimiento de Puntos
El Robot Pioneer es un robot diferencial con capacidad de navegación autónoma en entornos desconocidos. En este proyecto, se desarrolló un modelo de ecuaciones para un control de seguimiento de puntos utilizando MATLAB y Coppelia.

## Modelo de Ecuaciones
El control de seguimiento de puntos implica el movimiento del robot Pioneer para seguir una trayectoria especificada, que consiste en una secuencia de puntos en un plano 2D. El modelo de ecuaciones para el control de seguimiento de puntos en el Robot Pioneer se describe mediante las siguientes ecuaciones de cinemática:

**Cálculo de la distancia entre el robot Pioneer y el punto objetivo:**

d = sqrt((Xp-Xt)^2 + (Yp-Yt)^2)

Donde:
* d es la distancia entre el robot y el punto objetivo
* Xp y Yp son las coordenadas de la posición actual del robot Pioneer
* Xt y Yt son las coordenadas del punto objetivo

**Cálculo de la velocidad lineal deseada del robot:**

v = kpt * d

Donde:
* v es la velocidad lineal deseada del robot
* kpt es la constante de translación

**Cálculo del ángulo deseado hacia el punto objetivo:**

thetad = atan2(Yt - Yp, Xt - Xp)

Donde:
* thetad es el ángulo deseado hacia el punto objetivo
* Xp y Yp son las coordenadas de la posición actual del robot Pioneer
* Xt y Yt son las coordenadas del punto objetivo

**Cálculo de la velocidad angular del robot:**

w = -kpr * (θ - θd)

Donde:
* w es la velocidad angular del robot
* kpr es la constante de rotación
* theta es el ángulo actual del robot con respecto al eje X
* thetad es el ángulo deseado hacia el punto objetivo

**Cálculo de las velocidades individuales de las llantas del robot Pioneer:**

Vr = v + (L * ω) / 2;
Vl = v - (L * ω) / 2;

Donde:

* Vr y Vl son las velocidades lineales de las llantas derecha e izquierda, respectivamente
* v es la velocidad lineal deseada del robot
* L es la distancia entre las llantas del robot
* ω es la velocidad angular del robot

## Implementación en MATLAB y Coppelia
La implementación se lleva a cabo utilizando MATLAB y Coppelia. En MATLAB, se utilizan las API y las librerías disponibles para la comunicación con Coppelia. Se declaran los objetos necesarios, se calcula la posición del robot Pioneer, se obtienen los valores de las ecuaciones de cinemática, se calculan las velocidades individuales de las llantas y se envían a Coppelia para que el robot responda en consecuencia.

Para ejecutar el código, se debe tener instalado MATLAB y Coppelia, y se deben seguir las siguientes instrucciones:

1. Clonar o descargar el repositorio del proyecto desde GitHub.
2. Abrir MATLAB y configurar la conexión con Coppelia utilizando las API disponibles.
3. Ejecutar el script de control de seguimiento de puntos en MATLAB, que contiene la implementación del modelo de ecuaciones descrito anteriormente.
4. Iniciar la simulación en Coppelia para que el robot Pioneer comience a moverse según las velocidades y ángulos calculados en MATLAB.
5. Observar y analizar el comportamiento del robot Pioneer en la simulación, ajustando los valores de las constantes de translación y rotación (kpt y kpr) según sea necesario para obtener un seguimiento suave y preciso de la trayectoria de puntos.

Es importante tener en cuenta que este es un proyecto de ejemplo y que puede requerir ajustes y modificaciones según los requisitos específicos del entorno y las necesidades del proyecto. Se recomienda revisar y entender completamente el código y el modelo de ecuaciones antes de implementarlo en un entorno de producción o en un proyecto real.
