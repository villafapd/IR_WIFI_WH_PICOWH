#include <Arduino.h>
#include <stdlib.h>
#include <RP2040_PWM.h>
// Manejo de la terminal serie pero en UDP. Es necesario para depuerar el programa cuando se usa OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFi.h>


// ======================================================
// PINOUT
// ======================================================
#define GPIO00   0

// ======================================================
// Variables estáticas internas 
// ======================================================
unsigned long BaseTimer_00 = 5000;
unsigned long previousMillis = 0;
int freq = 38650;
int duty = 50;
bool Array_Code[160];     // ahora soporta hasta 160 bits
long registro[5];         // 5 * 32 bits = 160 bits
//int new_range = 255;
int totalBits;
bool UDP_Terminal = false;
String DirIP;
unsigned long ts;
bool comando_prev[32] = {0};   // Estado previo de cada comando
bool salida[32]       = {0};   // Salida one-shot de cada comando
long comandos_ir;

RP2040_PWM pwm(GPIO00, freq, duty);  // pin, frecuencia, duty %

// ======================================================
// CONFIGURACION WIFI
// ======================================================
const char* ssid = "RedWifi6_Mesh";
const char* password = "10242010";

// ======================================================
// CONFIGURACION MODBUS TCP
// ======================================================
const char* serverIP = "192.168.68.100";
const uint16_t serverPort = 15003;
uint16_t readRegs[20];
uint16_t writeRegs[20];

//Tabla registros asignables desde modbus TCP
bool Cmd_PowerOn;
bool Cmd_PowerOff; 

// ======================================================
// Tabla de codigos binarios del control remoto original
// Equipo de aire acondicionado WhestingHouse
// ======================================================
// Tabla de codigos IR
String Tabla_Codigos_IR[32] = 
{
  //PowerOn  
  "00000010000000000010100000000001110000100100000000010101000000101100010000000000000101100000000000000000100000000000000000000000001111000",
  //PowerOff
  "00000010000000000010100000000001110000101000000000010101000000101100000000000000000101100000000000000000100000000000000000000000100111000", 
  "Cadena2", 
  "Cadena3",
  "Cadena4", 
  "Cadena5", 
  "Cadena6", 
  "Cadena7",
  "Cadena8", 
  "Cadena9", 
  "Cadena10", 
  "Cadena11",
  "Cadena12", 
  "Cadena13", 
  "Cadena14", 
  "Cadena15",
  "Cadena16", 
  "Cadena17", 
  "Cadena18", 
  "Cadena19",
  "Cadena20", 
  "Cadena21", 
  "Cadena22", 
  "Cadena23",
  "Cadena24", 
  "Cadena25", 
  "Cadena26", 
  "Cadena27",
  "Cadena28", 
  "Cadena29", 
  "Cadena30", 
  "Cadena31"
};

//Configuración de anchos de pulsos IR
//Para Control Remoto WhestingHouse
int WH_Inicio = 3280;
int WH_Pausa = 1671;
int WH_UnoPrimeraParte = 396;
int WH_UnoSegundaParte = 1278;
int WH_CeroPrimeraParte = 396;
int WH_CeroSegundaParte = 435;


WiFiClient client;

// ======================================================
// PARAMETROS DE COMUNICACION MODBUS TCP/IP
// ======================================================
const uint16_t TIMEOUT_MS = 1000;             // Timeout de respuesta
const uint16_t RECONNECT_INTERVAL_MS = 2000;  // Tiempo entre reintentos
unsigned long lastReconnectAttempt = 0;
uint16_t transactionID = 1;
uint16_t UnitID = 8; //Numero Esclavo

// ======================================================
// PARAMETROS DE COMUNICACION UDP (Terminal)
// ======================================================
WiFiUDP Udp;
//Puerto y direccion IP del monitor remoto (Uso de UDP). En este caso es la raspberry PI4
IPAddress DelRemoto(192, 168, 68, 100);
int udpport = 15108;

// ======================================================
// Pulso One Shoot rising para múltiples comandos
// ======================================================
bool GenerarPulsoUnCiclo(uint8_t id, bool comando)
{
    // Detectar flanco ascendente: comando pasa de 0 → 1
    if (comando == 1 && comando_prev[id] == 0)
    {
        salida[id] = 1;   // Pulso de un ciclo
    }
    else
    {
        salida[id] = 0;   // En todos los demás casos, salida en 0
    }

    // Actualizar estado previo
    comando_prev[id] = comando;

    return salida[id];
}

// ======================================================
//  CONVERTIR CADENA BINARIA ? REGISTROS (hasta 160 bits)
// ======================================================
int binStringToRegistersAuto(const String &bin, long registro[])
{
    int len = bin.length();

    // Limitar a 160 bits
    if (len > 160)
        len = 160;

    // Calcular cuantos registros se necesitan
    int numReg = (len + 31) / 32;

    // Inicializar registros
    for (int i = 0; i < numReg; i++)
        registro[i] = 0;

    // Cargar bits en orden correcto (MSB ? LSB)
    for (int bit = 0; bit < len; bit++)
    {
        int regIndex = bit / 32;
        int offset   = bit % 32;

        if (bin[bit] == '1')
            registro[regIndex] |= (1L << offset);
    }

    return len;   // devuelve la cantidad real de bits cargados
}

// ======================================================
//  LEER UN BIT DE LOS REGISTROS (hasta 160 bits)
// ======================================================
bool bitOfWordVar(long registro[], int numBit, int totalBits)
{
    if (numBit < 0 || numBit >= totalBits)
        return false;

    int regIndex = numBit / 32;
    int offset   = numBit % 32;

    return (registro[regIndex] >> offset) & 1;
}

// ======================================================
// Enviar el codigo IR
// ======================================================
void CMD_IR_to_Ctrol(String Cadena, int inicio, int pausa, int unoprimeraparte, int unosegundaparte, int ceroprimeraparte, int cerosegundaparte)
{
        Serial.println("Cadena Entrada:" + Cadena);
        totalBits = binStringToRegistersAuto(Cadena, registro);
        Serial.print("Bits cargados: ");
        Serial.println(totalBits);

        Serial.println("Enviando Pulsos IR");
        // Inicio del comando IR
        pwm.setPWM(GPIO00, freq, duty);     // cambiar frecuencia y duty
        delayMicroseconds(inicio); 
        //Serial.print("1");

        // Pausa
        pwm.setPWM(GPIO00, freq, 0);     // cambiar frecuencia y duty
        delayMicroseconds(pausa);
        //Serial.print("0");

        // Envio del comando completo
        for (int i = 0; i < totalBits; i++)
        {
            Array_Code[i] = bitOfWordVar(registro, i, totalBits);
            if (Array_Code[i])
            {
                //----------------------------------------------------------------------------------------------------
                //Envio de 1(uno) que es la primera parte del pulso que forma el 1(uno) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, duty);     // cambiar frecuencia y duty
                delayMicroseconds(unoprimeraparte);
                //----------------------------------------------------------------------------------------------------
                //Envio de 0(cero) que es la segunda parte del pulso que forma el 1(uno) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, 0);     // cambiar frecuencia y duty
                delayMicroseconds(unosegundaparte);

                //Serial.print("1");
            }
            else
            {
                //----------------------------------------------------------------------------------------------------
                //Envio de 1(uno) que es la primera parte del pulso que forma el 0(cero) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, duty);     // cambiar frecuencia y duty
                delayMicroseconds(ceroprimeraparte); //396
                //----------------------------------------------------------------------------------------------------
                //Envio de 0(cero) que es la segunda parte del pulso que forma el 0(cero) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, 0);     // cambiar frecuencia y duty
                delayMicroseconds(cerosegundaparte);

                //Serial.print("0");
            }
        }

        //Serial.println();
        

}

// ======================================================
// Comando desde Modbus para ejectuar el comando IR
// ======================================================
void EjecutarComando(long Comando, String Array_Cadenas[32], int inicio, int pausa, int unoprimeraparte, int unosegundaparte, int ceroprimeraparte, int cerosegundaparte)
{
    for (int i = 0; i < 32; i++)
    {
        // Extraer bit i (0 = LSB)
        bool bit = (Comando >> i) & 0x01;

        if (bit)
        {
            Serial.println(String("Bit ") + i + " = 1 -> Ejecutando funcion CMD_IR_to_Ctrol con:");

            Serial.println(Array_Cadenas[i]);

            // Ejecutar comando IR con la cadena correspondiente
            CMD_IR_to_Ctrol(Array_Cadenas[i], inicio, pausa, unoprimeraparte, unosegundaparte, ceroprimeraparte, cerosegundaparte);
        }
        else
        {
            Serial.println(String("Bit ") + i + " = 0 -> No ejecuta");
        }
    }
}

// ======================================================
// Función para Monitorear a través de UDP remoto con OTA
// ======================================================
void UDP_Serial_Println(bool enable, String EntradaText)
{
 if (enable)
 {
  String str = EntradaText;
  //Construct Char* from String    
  str += "\n";
  int n = str.length(); 
  char text[n + 1];     
  strcpy(text, str.c_str()); 

  // String Texto = InputText;
  if (WiFi.status() == WL_CONNECTED)
    {
      Udp.beginPacket(DelRemoto, udpport);
      Udp.write(text);
      Udp.endPacket();
    }
  }
}

// ======================================================
// FUNCION: RECONEXION AUTOMATICA
// ======================================================
bool ensureConnection() {
    if (client.connected()) return true;

    unsigned long now = millis();
    if (now - lastReconnectAttempt < RECONNECT_INTERVAL_MS) return false;

    lastReconnectAttempt = now;

    Serial.println("Intentando reconectar Modbus TCP...");

    if (client.connect(serverIP, serverPort)) {
        Serial.println("Reconectado al servidor Modbus");
        return true;
    }

    Serial.println("Fallo de reconexion");
    return false;
}

// ======================================================
// FUNCION: VALIDAR TRANSACTION ID
// ======================================================
bool validateTID(uint8_t* response) {
    uint16_t respTID = (response[0] << 8) | response[1];

    if (respTID != transactionID) {
        Serial.printf("Transaction ID invalido. Esperado=%u, Recibido=%u\n",
                      transactionID, respTID);
        client.stop();
        return false;
    }
    return true;
}

// ======================================================
// FUNCION: LEER HOLDING REGISTERS (FC 0x03)
// ======================================================
bool modbusReadHolding(uint16_t startAddr, uint16_t count, uint16_t* buffer) {
    if (!ensureConnection()) return false;

    uint8_t request[12];

    request[0] = transactionID >> 8;
    request[1] = transactionID & 0xFF;
    request[2] = 0;
    request[3] = 0;
    request[4] = 0;
    request[5] = 6;
    request[6] = UnitID;        // Unit ID
    request[7] = 0x03;     // Function code
    request[8] = startAddr >> 8;
    request[9] = startAddr & 0xFF;
    request[10] = count >> 8;
    request[11] = count & 0xFF;

    client.write(request, 12);

    unsigned long start = millis();
    while (client.available() < 9 + count * 2) {
        if (millis() - start > TIMEOUT_MS) {
            Serial.println("Timeout en lectura Modbus");
            client.stop();
            return false;
        }
    }

    uint8_t response[256];
    client.read(response, sizeof(response));

    if (!validateTID(response)) return false;

    if (response[7] != 0x03) {
        Serial.println("Error: respuesta Modbus inesperada");
        return false;
    }

    for (int i = 0; i < count; i++) {
        buffer[i] = (response[9 + i * 2] << 8) | response[10 + i * 2];
    }

    transactionID++;
    return true;
}

// ======================================================
// FUNCION: ESCRIBIR HOLDING REGISTERS (FC 0x10)
// ======================================================
bool modbusWriteHolding(uint16_t startAddr, uint16_t count, uint16_t* values) {
    if (!ensureConnection()) return false;

    uint8_t request[260];

    uint16_t byteCount = count * 2;
    uint16_t length = 7 + byteCount;

    request[0] = transactionID >> 8;
    request[1] = transactionID & 0xFF;
    request[2] = 0;
    request[3] = 0;
    request[4] = length >> 8;
    request[5] = length & 0xFF;
    request[6] = UnitID;        // Unit ID
    request[7] = 0x10;     // Function code
    request[8] = startAddr >> 8;
    request[9] = startAddr & 0xFF;
    request[10] = count >> 8;
    request[11] = count & 0xFF;
    request[12] = byteCount;

    for (int i = 0; i < count; i++) {
        request[13 + i * 2] = values[i] >> 8;
        request[14 + i * 2] = values[i] & 0xFF;
    }

    client.write(request, 13 + byteCount);

    unsigned long start = millis();
    while (client.available() < 12) {
        if (millis() - start > TIMEOUT_MS) {
            Serial.println("Timeout en escritura Modbus");
            client.stop();
            return false;
        }
    }

    uint8_t response[12];
    client.read(response, 12);

    if (!validateTID(response)) return false;

    if (response[7] != 0x10) {
        Serial.println("Error: respuesta inesperada en FC16");
        return false;
    }

    transactionID++;
    return true;
}

// ======================================================
//  CONFIGURACION DE PINES
// ======================================================
void pinConfig()
{
    pinMode(GPIO00, OUTPUT);
    digitalWrite(GPIO00, LOW);
}

// ======================================================
//  SETUP
// ======================================================
void setup() 
{
    pinConfig();
    Serial.begin(115200);
    Serial.println("Booting");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        Serial.print(".");
    }

    Serial.println("\nWiFi conectado");
    Serial.println(WiFi.localIP());
    WiFi.persistent(true);
    

  //Port defaults to 8266
  ArduinoOTA.setPort(2040);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("IR_WIFI_AC01");
  // No authentication by default
  ArduinoOTA.setPassword("admin2");
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) 
    {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      //Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      //Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      //Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      //Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      //Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("OTA Operando");
  UDP_Serial_Println(UDP_Terminal, "IR_WIFI_AC01: OTA Operando");
  Serial.print("IP address: ");
  UDP_Serial_Println(UDP_Terminal, "IR_WIFI_AC01: IP address");
  Serial.println(WiFi.localIP());
  DirIP = WiFi.localIP().toString().c_str();
  UDP_Serial_Println(UDP_Terminal, DirIP);
  
  UDP_Serial_Println(UDP_Terminal, "IR_WIFI_AC01: Cliente modbus activo en PICO WH");    
}

// ======================================================
//  LOOP PRINCIPAL
// ======================================================
void loop()
{
/*
  if (millis() - previousMillis >= BaseTimer_00)
    {
        Serial.println("Cadena Entrada:");
        Serial.println(Cadena);

        totalBits = binStringToRegistersAuto(Cadena, registro);

        Serial.print("Bits cargados: ");
        Serial.println(totalBits);

        Serial.println("Inicio, Pausa y Cadena Salida:");

        // Inicio del comando IR
        pwm.setPWM(GPIO00, freq, duty);     // cambiar frecuencia y duty
        delayMicroseconds(3280); //3294
        //Serial.print("1");

        // Pausa
        pwm.setPWM(GPIO00, freq, 0);     // cambiar frecuencia y duty
        delayMicroseconds(1671);
        //Serial.print("0");

        // Envio del comando completo
        for (int i = 0; i < totalBits; i++)
        {
            Array_Code[i] = bitOfWordVar(registro, i, totalBits);
            if (Array_Code[i])
            {
                //----------------------------------------------------------------------------------------------------
                //Envio de 1(uno) que es la primera parte del pulso que forma el 1(uno) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, duty);     // cambiar frecuencia y duty
                delayMicroseconds(396);
                //----------------------------------------------------------------------------------------------------
                //Envio de 0(cero) que es la segunda parte del pulso que forma el 1(uno) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, 0);     // cambiar frecuencia y duty
                delayMicroseconds(1278);

                //Serial.print("1");
            }
            else
            {
                //----------------------------------------------------------------------------------------------------
                //Envio de 1(uno) que es la primera parte del pulso que forma el 0(cero) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, duty);     // cambiar frecuencia y duty
                delayMicroseconds(396); //396
                //----------------------------------------------------------------------------------------------------
                //Envio de 0(cero) que es la segunda parte del pulso que forma el 0(cero) del comando IR
                //----------------------------------------------------------------------------------------------------
                pwm.setPWM(GPIO00, freq, 0);     // cambiar frecuencia y duty
                delayMicroseconds(435);

                //Serial.print("0");
            }
        }

        //Serial.println();
        previousMillis = millis();
    }

*/
 
    //******************************************************************************************************************** 
    //Manejo de logica de OTA 
    //******************************************************************************************************************** 
    ArduinoOTA.handle();

    //******************************************************************************************************************** 
    //Polling Modbus TCP IP Cliente
    //******************************************************************************************************************** 

    // ---- Escritura FC16 ----
    if (modbusWriteHolding(10, 10, writeRegs)) 
    {
        Serial.println("Escritura FC16 OK");
    } else 
    {
        Serial.println("Error en FC16");
    }
    delay(2000);
    // ---- Lectura FC03 ----
    if (modbusReadHolding(0, 10, readRegs)) 
    {
        Serial.println("Lectura FC03 OK:");
        for (int i = 0; i < 10; i++) 
        {
            Serial.printf("Reg[%d] = %d\n", i, readRegs[i]);
        }
    } else 
    {
        Serial.println("Error en FC03");
    }
    delay(2000);


    //***************************************************
    //Tabla de datos compartidos con Modbus TCP Server (DomoServer) o simulador modbus
    //***************************************************
    //Datos recibidos desde el servidor Modbus
    //Aire Acondicionado WhestingHouse
    //UDP_Terminal = bool(readRegs[3]); //Hab. Terminal UDP
    Cmd_PowerOn = bitRead (readRegs[0],0); //0 = LSB, 15 = MSB en un entero de 16 bits
    Cmd_PowerOff = bitRead(readRegs[0],1); //0 = LSB, 15 = MSB en un entero de 16 bits

    UDP_Terminal= bool (readRegs[1]); // Hab. de la terminal para visualizar estados de variables de forma remota
    //Datos enviados al servidor Modbus
    //writeRegs[0]= 0;
    //writeRegs[1]= 0;
    //writeRegs[2]= 0;  
    //writeRegs[3]= 0;

    //******************************************************************************************************************** 
    //Comandos 
    //******************************************************************************************************************** 
    //Funcion OSR One Shoot Rising
    bool St_Cmd_PowerOn = GenerarPulsoUnCiclo(0, Cmd_PowerOn); //El cero en la funcion corresponde al guardado de la variables temporales
    if (St_Cmd_PowerOn)
    {
        Serial.println("Pulso Comando PowerOn");
        UDP_Serial_Println(UDP_Terminal,"Pulso Comando PowerOn" );
        // Ejecucion de un solo ciclo
        bitWrite(comandos_ir, 0, 1);
        EjecutarComando(comandos_ir,Tabla_Codigos_IR,WH_Inicio,WH_Pausa, WH_UnoPrimeraParte, WH_UnoSegundaParte, WH_CeroPrimeraParte, WH_CeroSegundaParte);
        bitWrite(comandos_ir, 0, 0);
    }
    //Funcion OSR One Shoot Rising
    bool St_Cmd_PowerOff = GenerarPulsoUnCiclo(1, Cmd_PowerOff); //El uno en la funcion corresponde al guardado de la variables temporales
    if (St_Cmd_PowerOff)
    {
        Serial.println("Pulso Comando PowerOff");
        UDP_Serial_Println(UDP_Terminal,"Pulso Comando PowerOff" );
        // Ejecucion de un solo ciclo
        bitWrite(comandos_ir, 1, 1);
        EjecutarComando(comandos_ir,Tabla_Codigos_IR, WH_Inicio,WH_Pausa, WH_UnoPrimeraParte, WH_UnoSegundaParte, WH_CeroPrimeraParte, WH_CeroSegundaParte);
        bitWrite(comandos_ir, 1, 0);
    }



}