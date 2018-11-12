# Pinturillo
Código Arduino para el Robot Siguelíneas "Pinturillo".

## El Robot
Pinturillo es un robot sigue-líneas basado en Arduino, que utiliza un par de servos como motores y un par de sensores LDR. Su código utiliza estados para determinar el comportamiento del robot, así como un PID para estabilizarlo. Algunos detalles de la construcción del mismo se pueden leer en [mi blog personal](https://miguel-angel-garcia.com/2018/robot-pinturillo/).

## Versiones del Software de Control
En este repositorio hay dos versiones del código de Pinturillo: uno de ellas [sin PID](https://github.com/miguel-angel-garcia/pinturillo/blob/master/Robot_Siguelineas_Pinturillo_v2.ino) y la otra usando un [PID muy simple](https://github.com/miguel-angel-garcia/pinturillo/blob/master/Robot_Siguelineas_Pinturillo_PID.ino). La utilizada en la versión final del robot ha sido la última, es decir, con PID.

## Competición en la Cateforía de Sigue-líneas la Edición 2018 de la OSHWDem
Pinturillo ha competido "barriendo a sus seguidores" ;-) en la edición 2018 de la [OSHWDem](https://oshwdem.org/) obteniendo el primer puesto en la categoría de sigue-líneas.
