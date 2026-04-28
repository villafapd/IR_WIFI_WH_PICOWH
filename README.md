Ingenieria inversa de un control remoto Infrarrojos
Con ayuda de un osciloscopio se puede capturar el tren de pulsos que emite un control remoto de infrarrojos.
Luego de la captura se analiza la frecuencia de modulación, el ancho de pulsos para inicio, pausa, uno y cero.
Posteriormente se imita el tren de pulsos desde un placa microcontroladora Raspberry Pi pico WH para poder controlar el dispositivo que tiene el control remoto IR 
Finalmente se le agrega a la placa los el protocolo de comunicaciones modbus tcp / ip para su control remoto via Wifi desde un servidor modbus tcp / ip, como tambien se le agregó OTA para su programación en forma remota desde VSCode con la extensión PlatFormio.
