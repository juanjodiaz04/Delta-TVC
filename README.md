# Delta-TVC
Repositorio para el proyecto del curso de Electrónica Digital III

## Idea de Proyecto
El proyecto a realizar que se plantea es el diseño de un controlador de tipo PID (puede ser PI) para ángulo de empuje de un motor de cohete. En este caso por facilidad no sé realiza con propelente sólido, y se propone como alternativa para simular el empuje un motor brushless (y su ESC). Para medir la variable de control se utilizará una IMU y para realizar el control se utilizan servomotores que estabilizan el ángulo de inclinación. 

Adicionalmente se hará una interfaz de usuario empleando una pantalla LCD y teclado(medir viabilidad) que permita modificar los parámetros del controlador y muestre cuando el sistema se encuentra por fuera del rango del setpoint. 

Finalmente se miden variables significativas del sistema como: consumo de energía, voltaje, entre otros.

## Motivación
La motivación personal de este proyecto surge del desafío que representa la implementación de controladores PID en sistemas que interactúan de manera compleja con el entorno. Complementado con una curiosidad por aprender a desarrollar un sistema embedido y observar sus aplicaciones en la vida real. Además de que existe un interés **colectivo** dentro del equipo por los temas aeroespaciales y la cohetería.

## Requisitos No Funcionales
- **Rendimiento**: El sistema embebido debe mantener la estabilidad del ángulo del cohete dentro de un margen de error máximo del 5% respecto al valor objetivo (*setpoint*).
-	**Fiabilidad**: El sistema debe ser capaz de responder adecuadamente ante perturbaciones angulares de hasta 30 grados, garantizando un comportamiento controlado y seguro.
-	**Usabilidad**: Se debe proporcionar una interfaz de usuario intuitiva que permita visualizar en tiempo real el ángulo actual del sistema, así como ingresar de forma sencilla los parámetros del controlador PID.
-	**Tiempo de respuesta**: El sistema debe ser capaz de reaccionar ante una perturbación y estabilizar el ángulo en un tiempo inferior a 3 segundos.
-	**Consumo de Energía**: Considerando los tiempos y consumo del vuelo del cohete, el sistema debe estar encendido por lo menos 20 minutos durante el vuelo, y 5 minutos de control activo.
