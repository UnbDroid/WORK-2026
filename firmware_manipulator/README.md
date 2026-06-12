# Firmware Manipulador #
Esse código deve ser compilado no PlatformIO\
É necessário importar a biblioteca "Servo" (by Michael Margolis) para que ele funcione\
Para fazer os motores de passo funcionarem, utilize a função setpTo, fazendo com um dos motores ou os dois motores andem x passos em uma certa direção com uma certa velocidade\
O formato é **stepTo(int steps, bool direction, int time, int motor)**, onde:
- **steps**: passos para o motor dar (0-200)
- **direction**: direção para o motor girar (true para horário, false para anti-horário)
- **time**: tempo de espera entre os passos (em microssegundos, por exemplo, 2000us)
- **motor**: escolha do motor que vai girar (0 = um motor, 1 = outro motor, 2 = os dois motores juntos)
