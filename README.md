# Robot de desinfección para COVID-19
_Proyecto impulsado por ETSII-CR (Escuela Superior de Ingeniería Industrial - Ciudad Real)_

Se trata de un robot contruido sobre la base _kobuki_ y montados unos tubos _UVC_ que desinfectarán las áreas proximas.

Se usará ROS-MELODIC para la realización de este proyecto.

## Comenzando 🚀

### Prerequisitos
asdf

### Comandos para la instalacion de los paquetes necesarios
Creamos un directorio de trabajo:
```
$ mkdir -p ~/catkin_ws/src/
```

Nos movemos dentro del directorio, lo compilamos y lo añadimos al bashrc:
```
$ cd ~/catkin_ws/
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

Descargamos los paquetes necesarios (los que no estan incluidos por defecto con apt)
```
$ rosinstall ~/catkin_ws/src https://raw.githubusercontent.com/TheBerbat/uvone_robot/devel/uvone.rosinstall
```
Resolvemos las dependencias:
```
$ rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y
```

El mensaje devuelto deverá ser:
```
#All required rosdeps installed successfully
```
En caso contrario deberá abrirse un issue con la mayor aportación de información posible.


## Fotos
Actualmente el README.md se encuentra en fase de construcción...
