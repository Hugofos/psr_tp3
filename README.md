# psr_tp3
Trabalho realizado por
- Tiago Pereira 98360
- Hugo Silva 86157
- Diogo Lapa 117296

Este trabalho foi feito no ambito da cadeira Programação de Sistemas Robóticos, sendo o terceiro destes trabalhos para avaliação.

O trabalho consiste no desenvolvimento de um sistema robótico que funcione como um mordomo, utilizando ROS associado ao ambiente de simulação gazebo e rviz, sendo que iremos simular a utilização do robô turtlebot3 - Waffle_pi.

Esta simulação passa-se no cenário AWS small house e tem como objetivo o mapeamento do cenário através da movimentação do robô pelo apartamento. Estes movimentos podem ser feitos de diversas formas, por condução manual, por consução autónoma com alvo fixo ou através de informação passada pelo terminal. Aliado a isto, o robô deverá ainda executar um determinado número de missões pré-definidas e sincronamente analisar o ambiente ao seu redor e detetar objetos ao longo do seu trajeto.

Para executar este nosso programa, deverá:

- Carregar o simulador Gazebo com o devido mapa
  ```bash
    roslaunch psr_tp3 gazebo.launch
  ```
- Carregar o robô waffle_pi para o mapa
    ```bash
    roslaunch psr_tp3 bringup.launch
  ```

