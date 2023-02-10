# FINDER VERSÃO 3.0

Este pacote fornece o FINDER_V3,


# Ambiente de desenvolvimento：
- Ubuntu 18.04
- ROS Melodic
- Python>=3.6.0

# Pré-requisitos:


## Instalando FINDER-V3

### 1. Clone os arquivos para a Workspace de sua preferência:

```
cd /your/catkin_ws/src

git clone https://github.com/fabioabdon/Finder-3.0.git

cd yolov5_ros/yolov5
```
### 2. Instale os pre-requisitos no ambiente virtual criado

```
sudo pip install -r requirements.txt
```

## Ajustando os parâmetros

1. Certifique-se de colocar seus pesos na pasta [weights](https://github.com/fabioabdon/YOLOV5-ROS/tree/main/yolov5_ros/yolov5_ros/weights). 
2. Caso o arquivo dos pesos estejam com um nome diferente, altere-o em `launch/yolo_v5.launch`.
3. Altere o tópico da imagem que deseja utilizar em `launch/yolo_v5.launch`.

## Iniciar yolov5_ros

```
roslaunch yolov5_ros yolo_v5.launch
```


## Parâmetros do nó

* **`image_topic`** 

    Tópico de câmera inscrito.

* **`weights_path`** 

    Caminho para o arquivo de pesos.

* **`pub_topic`** 

    Tópico publicado com as caixas delimitadoras detectadas.
    
* **`confidence`** 

    Limite de confiança para objetos detectados.
    
