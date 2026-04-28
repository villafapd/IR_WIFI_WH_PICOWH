Ingenieria inversa de un control remoto Infrarrojos

<img width="1536" height="1024" alt="Copilot_20260427_211617" src="https://github.com/user-attachments/assets/63601016-e711-44e8-8e86-dbf239506274" />

Con ayuda de un osciloscopio se puede capturar el tren de pulsos que emite un control remoto de infrarrojos.

<img width="7459" height="4164" alt="20260428_142343_hor" src="https://github.com/user-attachments/assets/f00f9aec-6553-480c-b3fa-dcb705a2d95e" />


Luego de la captura se analiza la frecuencia de modulación, el ancho de pulsos para inicio, pausa, uno y cero.

<img width="1357" height="558" alt="image" src="https://github.com/user-attachments/assets/0e260dda-0394-46fc-9552-d73613f00a73" />

Posteriormente se imita el tren de pulsos desde un placa microcontroladora Raspberry Pi pico WH para poder controlar el dispositivo que tiene el control remoto IR 

<img width="6266" height="4164" alt="20260428_143227" src="https://github.com/user-attachments/assets/6edf1268-9546-4776-813d-cdc54a60e01d" />

Finalmente se le agrega a la placa los el protocolo de comunicaciones modbus tcp / ip para su control remoto via Wifi desde un servidor modbus tcp / ip, como tambien se le agregó OTA para su programación en forma remota desde VSCode con la extensión PlatFormio.
