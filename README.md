# FINDER VERSÃO 3.0

Este pacote fornece o FINDER_V3, um gerador de ZONA PROXÊMICA individual que, junto com o YOLO V5, detecta a posição do obstaculo no ambiente e gera a zona proxêmica daquele respectivo obstáculo. 


# Ambiente de desenvolvimento：
- Ubuntu 18.04
- ROS Melodic
- Python>=3.6.0


Por estarmos utilizando o ROS Melodic, o catkin_make apenas compila scripts Python 2, por isso clonaremos uma nova WORKSPACE especificamente para rodar Python 3.

# Tópicos de leitura e escrita necessários:

## Leitura
- Bounding box (X_max, Y_max, X_min, Y_min)
- Localização do robô em tempo real (amcl_pose)

## Escrita
- People


# Pré-requisitos:

## Dependências:

```
sudo apt-get install python3-pip python-catkin-tools python3-dev python3-numpy 

sudo pip3 install rospkg catkin_pkg
```

## Instalando FINDER-V3

### 1. Clone os arquivos para sua Pasta Pessoal:

```
git clone https://github.com/fabioabdon/Finder-3.0.git
```
### 2. Compilação

```
cd ~/cvbridge_build_ws 

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/x86_64-linux-gnu/libpython3.6m.so 

catkin config --instalar
```

NOTA: Altere o caminho do python de acordo, você pode verificar o local via Python 3.

```
python3 

import sys

print(sys.executable) #Print python3 executável path

print(sys.path) #Imprimir caminho da biblioteca python3

python3-config --includes #Arquivos de inclusão
```

Após a conclusão da configuração, crie o pacote:


```
catkin construir cv_bridge
```

### 3. Clonando Plugin
Para que o Finder funcione, será necessário baixar o plugin na Workspace onde está localizado o robô. O tutorial pode ser encontrado [aqui](https://github.com/iml130/proxemic_layer). 

## Ajustando os parâmetros

1. Altere o tópico da imagem de profundidade que deseja utilizar em `Finder_3.0/finder_v3.0.py`.


## Inicializando Finder V3

```
cd Finder-3.0

source install/setup.bash --extend

cd src/Finder_2.0/

python3 finder_v3.0.py
```

## Referencia para criar workspace no ROS com Python3
https://cyaninfinite.com/ros-cv-bridge-with-python-3/

