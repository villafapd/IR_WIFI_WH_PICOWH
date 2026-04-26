#include <Arduino.h>
#include <stdlib.h>
#include <RP2040_PWM.h>
// Manejo de la terminal serie pero en UDP. Es necesario para depuerar el programa cuando se usa OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFi.h>




// PINOUT
#define GPIO00   0
unsigned long BaseTimer_00 = 5000;
unsigned long previousMillis = 0;
int freq = 38650;
int duty = 50;
bool Array_Code[160];     // ahora soporta hasta 160 bits
long registro[5];         // 5 * 32 bits = 160 bits
//int new_range = 255;
// EJEMPLO DE CADENA (PUEDE SER DE 32 A 160 BITS)
String PowerOn =  "00000010000000000010100000000001110000100100000000010101000000101100010000000000000101100000000000000000100000000000000000000000001111000";
String PowerOff = "00000010000000000010100000000001110000101000000000010101000000101100000000000000000101100000000000000000100000000000000000000000100111000";
String Cadena = PowerOff;
int totalBits;

String DirIP;
unsigned long ts;

RP2040_PWM pwm(GPIO00, freq, duty);  // pin, frecuencia, duty %

// ---------------------------
// CONFIGURACI�N WIFI
// ---------------------------
const char* ssid = "RedWifi6_Mesh";
const char* password = "10242010";

// ---------------------------
// CONFIGURACI�N MODBUS TCP
// ---------------------------
const char* serverIP = "192.168.68.100";
const uint16_t serverPort = 15003;
uint16_t readRegs[20];
uint16_t writeRegs[20];

WiFiClient client;

// ---------------------------
// PAR�METROS DE COMUNICACI�N
// ---------------------------
const uint16_t TIMEOUT_MS = 1000;             // Timeout de respuesta
const uint16_t RECONNECT_INTERVAL_MS = 2000;  // Tiempo entre reintentos
unsigned long lastReconnectAttempt = 0;

uint16_t transactionID = 8;

WiFiUDP Udp;

//Puerto y direccion IP del monitor remoto (Uso de UDP). En este caso es la raspberry PI4
IPAddress DelRemoto(192, 168, 68, 100);
int udpport = 15108;

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Funci�n para Monitorear a traves de UDP remoto con OTA
void UDP_Serial_Println(String EntradaText)
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
    request[6] = 1;        // Unit ID
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
    request[6] = 1;        // Unit ID
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

// -----------------------------------------------------------------------------
//  CONVERTIR CADENA BINARIA ? REGISTROS (hasta 160 bits)
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
//  LEER UN BIT DE LOS REGISTROS (hasta 160 bits)
// -----------------------------------------------------------------------------
bool bitOfWordVar(long registro[], int numBit, int totalBits)
{
    if (numBit < 0 || numBit >= totalBits)
        return false;

    int regIndex = numBit / 32;
    int offset   = numBit % 32;

    return (registro[regIndex] >> offset) & 1;
}

// -----------------------------------------------------------------------------
//  CONFIGURACION DE PINES
// -----------------------------------------------------------------------------
void pinConfig()
{
    pinMode(GPIO00, OUTPUT);
    digitalWrite(GPIO00, LOW);
}

// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------
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
  UDP_Serial_Println("IR_WIFI_AC01: OTA Operando");
  Serial.print("IP address: ");
  UDP_Serial_Println("IR_WIFI_AC01: IP address");
  Serial.println(WiFi.localIP());
  DirIP = WiFi.localIP().toString().c_str();
  UDP_Serial_Println(DirIP);
  
  UDP_Serial_Println("IR_WIFI_AC01: Cliente modbus activo en PICO WH");    
}

// -----------------------------------------------------------------------------
//  LOOP PRINCIPAL
// -----------------------------------------------------------------------------
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

  
    // Valores de prueba para escritura
    for (int i = 0; i < 10; i++) writeRegs[i] = i + 1000;

    // ---- Escritura FC16 ----
    if (modbusWriteHolding(0, 10, writeRegs)) {
        Serial.println("Escritura FC16 OK");
    } else {
        Serial.println("Error en FC16");
    }

    delay(1000);

    // ---- Lectura FC03 ----
    if (modbusReadHolding(0, 10, readRegs)) {
        Serial.println("Lectura FC03 OK:");
        for (int i = 0; i < 10; i++) {
            Serial.printf("Reg[%d] = %d\n", i, readRegs[i]);
        }
    } else {
        Serial.println("Error en FC03");
    }

    delay(2000);


   



      //***************************************************
      //Tabla de datos compartidos con Modbus
      //***************************************************
      //Datos recibidos desde el servidor Modbus
      //Cmd_Modbus_LuzCocina = bool (Read_MB_Reg[0]);
      
      //Datos enviados al servidor Modbus
      //Write_MB_Reg[0]= digitalRead(PIN_RELE_COCINA);
      //Write_MB_Reg[1]= int(Cmd_Modbus_LuzCocina);
      //Write_MB_Reg[2]= Cmd_Luz_Cocina;  
      //Write_MB_Reg[3]= int(triggerFanLogic);

}