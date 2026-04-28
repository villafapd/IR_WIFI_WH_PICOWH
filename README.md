Ingenieria inversa de un control remoto Infrarrojos

<img width="1536" height="1024" alt="Copilot_20260427_211617" src="https://github.com/user-attachments/assets/63601016-e711-44e8-8e86-dbf239506274" />

Con ayuda de un osciloscopio se puede capturar el tren de pulsos que emite un control remoto de infrarrojos.

<img width="9248" height="4164" alt="20260428_142343_hor" src="https://github.com/user-attachments/assets/ee5c7aae-991c-404c-8d43-397614a47ada" />

Luego de la captura se analiza la frecuencia de modulación, el ancho de pulsos para inicio, pausa, uno y cero.
Posteriormente se imita el tren de pulsos desde un placa microcontroladora Raspberry Pi pico WH para poder controlar el dispositivo que tiene el control remoto IR 
Finalmente se le agrega a la placa los el protocolo de comunicaciones modbus tcp / ip para su control remoto via Wifi desde un servidor modbus tcp / ip, como tambien se le agregó OTA para su programación en forma remota desde VSCode con la extensión PlatFormio.
