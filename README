# Yolov5_ros

Este pacote fornece um wrapper ROS para PyTorch-YOLOv5 baseado em PyTorch-YOLOv5. O pacote foi testado no Ubuntu 18.04.

<p>
   <img width = "1000" src="https://github.com/qq44642754a/Yolov5_ros/blob/master/yolov5_ros/yolov5_ros/media/image.png"></a>
</p>


# Ambiente de desenvolvimento：
- Ubuntu 18.04
- ROS Melodic
- Python>=3.6.0
- PyTorch>=1.7

# Pré-requisitos:

## Instalar Anaconda:

### 1. Primeiro baixe o pacote de instalação correspondente [Anaconda](https://www.anaconda.com/products/individual#linux)
### 2. Em seguida, instale o Anaconda (por exemplo):

```
bash ~/Downloads/Anaconda3-2022.10-Linux-x86_64.sh
```
### 3. Edite o arquivo ~/.bashrc adicionando ao final dele:

```
export PATH=/home/your/anaconda3/bin:$PATH
```
### 4. Execute após salvar e sair:

```
source ~/.bashrc
```

## Instalar Pytorch:

### 1. Primeiro crie um ambiente virtual anaconda para pytorch:
Altere o nome mypytorch para o nome de sua preferência. 

```
conda create -n mypytorch python=3.8
```
### 2. Ative o ambiente mypytorch:

```
conda activate mypytorch
```
### 3. Instale o pytorch1.8 no ambiente pytorch criado
Instalar PyTorch: https://pytorch.org/get-started/locally/
```
conda install pytorch torchvision cudatoolkit=10.2 -c pytorch
```
### 4. Edite o arquivo ~/.bashrc, defina para usar python3.8 no ambiente mypytorch

```
alias python='/home/your/anaconda3/envs/mypytorch/bin/python3.8'
```
### 5. Execute após salvar e sair:

```
source ~/.bashrc
```

## Instalando yolov5_ros

### 1. Clone os arquivos para a Workspace de sua preferência:

```
cd /your/catkin_ws/src

git clone https://github.com/fabioabdon/YOLOV5-ROS.git

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
    
