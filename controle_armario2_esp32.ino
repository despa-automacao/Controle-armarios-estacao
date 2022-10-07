//7,7 segundos pra sair do setup

#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Wire.h>
#include "RTClib.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include <esp_now.h>
unsigned long tempo_random = 0;

bool pronto_envio = false;
//Definindo estrutura de dados para o ESP-Now
typedef struct struct_message {
  int identificador;
  float msgTemp;
  float msgUmi;
  double msgCo2;
  int msgDia;
  int msgMes;
  int msgAno;
  int msgHora;
  int msgMin;
  int msgSeg;
} struct_message;
//instanciando estrutura
struct_message message;

//Endereço MAC do par 0C:B8:15:C3:3F:B4
uint8_t broadcastAddress[] = {0x0C, 0xB8, 0x15, 0xC3, 0x3F, 0xB4};

//Informações do par
esp_now_peer_info_t peerInfo;

//Funçção de callback para o envio de mensagens
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r \n Last Packet Send Status: \t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS ? "Delivery Sucess" : "Delivery Fail");
}


//define modelo do RTC
RTC_DS1307 rtc;

//variaveis para o teclado de membrana
#define ROW_NUM     4 // four rows
#define COLUMN_NUM  4 // four columns

//variavel do tamanho da EEPROM que vai se utilizada
#define EEPROM_SIZE 18 //tres para os numeros inteiros e tres para os decimais 

//define dias da semana para o rtc usar
char diasDaSemana[7][12] = {"Domingo", "Segunda", "Terca", "Quarta", "Quinta", "Sexta", "Sabado"};

//variaveis que armazenam os setpoints e auxiliares
float setpoint_temp, setpoint_umi, setpoint_co2;
int temp_dec, umi_dec, co2_dec, temp_dec1, umi_dec1, co2_dec1, temp_dec2, umi_dec2, co2_dec2;
int temp_int, umi_int, co2_int, temp_int1, umi_int1, co2_int1, temp_int2, umi_int2, co2_int2;
long aux_int;
int aux_dec;

//variaveis para a iniciar o LCD
int lcdColumns = 20;
int lcdRows = 4;

//variaveis da tela
bool tela_setpoint = false;
bool tela_setpoints_enviados = false;

//variaveis para mudanca do setpoint
int seletor = 0;
int incrementador_salva = 0;
bool altera_rapido = false;

//salva millis
unsigned long salva_click = 0;
unsigned long att_tela = 0;
unsigned long salva_setpoints_enviados = 0;
unsigned long salva_rtc = 0;
unsigned long salva_sd = 60000;
unsigned long tempo_envia_reset = 0;
unsigned long envio_forcado_setpoint = 0;
unsigned long tempo_bt = 0;

//variaveis para enviar os setpoints para o MEGA
long setpoint_temp_int;
long setpoint_umi_int;
long setpoint_co2_int;
char bufferfinal[17];

//variaveis que armezanam a hora e data
int dia, mes, ano, hora, minuto, segundo;

//variaveis do display para monstar as leituras e auxiliares
float temp, umi, co2;
long leitura_temp, leitura_temp2, leitura_umi, leitura_umi2, leitura_co2;

//variaveis para leitura da serial2, contem leituras de maquina
char mensagem[40];
int posicao = 0;

//variaveis para salvar os dados no cartao sd
char salvamento[40] = "0000,0000,000000,00:00:00,00/00/0000,";

//variavel de envio dos setpoint forcadamente
bool envio_forcado = true;

//variavel que checa o envio dos setpoints
bool foi_enviado = false;

//variavel de envio dos setpoins assim que o controlador liga para o arduino mega
bool passe_envio = true;

//variaveis de salvar as leituras e evitar prints desnecessarios no display
float temp_backup = 28;
float umi_backup, co2_backup;

//define os parametros do LCD
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);


//start = A //open = 1 //reset = 4 //right = 8 //setpoint = 0
//pause = 3  //return = B //left = C //program = 7 //menu = *
//stop = 2 //down = 6 //ok = 9 //up = #
char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pin_rows[ROW_NUM]      = {13, 14, 27, 26}; // GIOP19, GIOP18, GIOP5, GIOP17 connect to the row pins
byte pin_column[COLUMN_NUM] = {25, 32, 15, 12};   // GIOP16, GIOP4, GIOP0, GIOP2 connect to the column pins

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  //WIFI para station
  WiFi.mode(WIFI_STA);
  //Inicializando ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //Registrando função de callback
  esp_now_register_send_cb(OnDataSent);
  //Registrando Par
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  //Adicionando par
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  delay(100);
  //inicia lcd e liga a luz de fundo
  lcd.begin();
  lcd.backlight();
  delay(100);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("INICIANDO");
  //Serial.println("Iniciando...");
  if (! rtc.begin()) {                         //Se o RTC nao for inicializado, faz
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO RTC");
  }
  delay(5000);


  //iniciar a EEPROM e reportar erro caso necessario
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO EEPROM");
    while (1) {
      //laço infinito pra barrar o funcionamento se a eeprom falhar
    }
  }
  //leituras da EEPROM e armazenamento dos setpoints
  temp_int = EEPROM.read(0);
  temp_dec = EEPROM.read(1);
  umi_int = EEPROM.read(2);
  umi_dec = EEPROM.read(3);
  co2_int = EEPROM.read(4);
  co2_dec = EEPROM.read(5);
  temp_int1 = EEPROM.read(6);
  temp_dec1 = EEPROM.read(7);
  umi_int1 = EEPROM.read(8);
  umi_dec1 = EEPROM.read(9);
  co2_int1 = EEPROM.read(10);
  co2_dec1 = EEPROM.read(11);
  temp_int2 = EEPROM.read(12);
  temp_dec2 = EEPROM.read(13);
  umi_int2 = EEPROM.read(14);
  umi_dec2 = EEPROM.read(15);
  co2_int2 = EEPROM.read(16);
  co2_dec2 = EEPROM.read(17);
  if ((temp_int != temp_int1) && (temp_int1 != temp_int2) && (temp_int != temp_int2)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO EEPROM");
    while (1) {
    }
  }

  if ((temp_dec != temp_dec1) && (temp_dec1 != temp_dec2) && (temp_dec != temp_dec2)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO EEPROM");
    while (1) {
    }
  }

  if ((umi_int != umi_int1) && (umi_int1 != umi_int2) && (umi_int != umi_int2)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO EEPROM");
    while (1) {
    }
  }

  if ((umi_dec != umi_dec1) && (umi_dec1 != umi_dec2) && (umi_dec != umi_dec2)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO EEPROM");
    while (1) {
    }
  }

  if ((co2_int != co2_int1) && (co2_int1 != co2_int2) && (co2_int != co2_int2)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO EEPROM");
    while (1) {
    }
  }

  if ((co2_dec != co2_dec1) && (co2_dec1 != co2_dec2) && (co2_dec != co2_dec2)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO EEPROM");
    while (1) {
    }
  }

  setpoint_temp = temp_int + (temp_dec * 0.1);
  setpoint_umi = umi_int + (umi_dec * 0.1);
  setpoint_co2 = co2_int + (co2_dec * 0.1);

  //  Set the RTC Time to 5:10:30 Nov 3 2020
  //rtc.adjust(DateTime(2021,12,21,11,17,0));

  //inicialização do cartao sd
  if (!SD.begin()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERRO CARTAO SD");
    while (1) {

    }
  }

  //criacao do arquivo D0



  File file = SD.open("/d0.txt");
  //verifica a existencia do arquivo, se nao existir cria outro
  if (!file) {
    //arquivo nao existe e sera criado
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CRIANDO ARQUIVO TXT");
    writeFile(SD, "/d0.txt", "TEMPERATURA,UMIDADE,CO2,HORA,DATA,");
    appendFile(SD, "/d0.txt", "\r\n");
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ARQUIVO TXT");
    lcd.setCursor(0, 1);
    lcd.print("JA EXISTE");
    appendFile(SD, "/d0.txt", "NOVA LEITURA");
    appendFile(SD, "/d0.txt", "\r\n");

  }
  file.close();
  delay(2500);




}

void loop() {
  while (Serial2.available() >= 1) {
    char recebido = Serial2.read();
    if (recebido != ',' && recebido != '0' && recebido != '1' && recebido != '2' && recebido != '3' && recebido != '4' && recebido != '5' && recebido != '6' && recebido != '7' && recebido != '8' && recebido != '9' && recebido != '\n' && recebido != '\0') {
      posicao = 0;
    }
    else {
      if (recebido != '\n'  /*&&(posicao < 20 - 1)*/) {
        mensagem[posicao] = recebido;
        posicao++;
      }
      else {
        mensagem[posicao] = '\0';
        posicao = 0;
        Serial.println(mensagem);
        long inutil;
        sscanf(mensagem, "%ld,%ld,%ld,%ld,%ld,%ld", &leitura_temp, &leitura_umi, &leitura_temp2, &leitura_umi2, &leitura_co2, &inutil);

        Serial.println(leitura_co2);
        temp = float((leitura_temp + leitura_temp2) / 2);
        umi = float((leitura_umi + leitura_umi2) / 2);
        co2 = float(leitura_co2);
        co2 = co2 / 10000;



        temp = temp / 100.00;
        umi = umi / 100.00;

        Serial.print(temp); Serial.print("\t");
        Serial.print(umi); Serial.print("\t");
        Serial.println(co2);

      }
    }
  }



  if (temp < 5.0 || temp > 60.0 || umi < 5.0 || umi > 100 || co2 < 0.04) {
    temp = temp_backup;
    umi = umi_backup;
    co2 = co2_backup;
  }
  else {
    temp_backup = temp;
    umi_backup = umi;
    co2_backup = co2;
    if ((millis() - tempo_bt) > 30000000) {
      message.msgTemp = temp;
      message.msgUmi = umi;
      message.msgCo2 = co2;
      message.identificador = 2;
      message.msgDia = dia;
      message.msgMes = mes;
      message.msgAno = ano;
      message.msgHora = hora;
      message.msgMin = minuto;
      message.msgSeg = segundo;
      pronto_envio = 1;
      tempo_bt = millis();
    }
  }

  //enviando mensagem via esp NOW
  if (pronto_envio == true) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &message, sizeof(message));
    pronto_envio = false;
    tempo_bt = millis();
    if (result == ESP_OK) Serial.println("Sending confirmed");
    else Serial.println("Sending failed");
  }

  if (millis() - salva_rtc >= 1000) {
    DateTime agora = rtc.now();                          // Faz a leitura de dados de data e hora
    dia = agora.day(), DEC; //armazena dia
    mes = agora.month(), DEC; //armazena mes
    ano = agora.year(), DEC; //armazena ano
    hora = agora.hour(), DEC; //armazena hora
    minuto = agora.minute(), DEC; //armazena minuto
    segundo = agora.second(), DEC; //armazena segundo
    salva_rtc = millis();
  }
  char key = keypad.getKey();
  if (key) {
    key = key;
  }
  else {
    key = 'x';
  }
  if (key == 'A') {
    Serial2.write('a');
    Serial2.write('\n');
  }

  if (key == '3') {
    Serial2.write('b');
    Serial2.write('\n');
  }

  if (key == '2') {
    Serial2.write('c');
    Serial2.write('\n');
  }

  if (key == '1') {
    Serial2.write('d');
    Serial2.write('\n');
  }

  if (key == '0') {
    tela_setpoint = !tela_setpoint; //habilita ou desabilita a programacao dos setpoints
  }
  if (key == '#') { //tecla que altera o seletor dos setpoints
    seletor--;
    if (seletor == -1) {
      seletor = 2;
    }
  }
  if (key == '6') { //tecla que altera o seletor dos setpoints
    seletor++;
    if (seletor == 3) {
      seletor = 0;
    }
  }

  if (key == 'C') { //tecla que diminui o setpoint
    if (seletor == 0) {
      if (millis() - salva_click < 500) {
        incrementador_salva++;
        if (incrementador_salva > 10) {
          altera_rapido = true;
        }
      }
      if (millis() - salva_click > 500) {
        incrementador_salva = 0;
        altera_rapido = false;
      }
      salva_click = millis();
      if (altera_rapido == true) {
        setpoint_temp = setpoint_temp - 1;
      }
      else {
        setpoint_temp = setpoint_temp - 0.1;
      }
    }
    if (seletor == 1) {
      if (millis() - salva_click < 500) {
        incrementador_salva++;
        if (incrementador_salva > 10) {
          altera_rapido = true;
        }
      }
      if (millis() - salva_click > 500) {
        incrementador_salva = 0;
        altera_rapido = false;
      }
      salva_click = millis();
      if (altera_rapido == true) {
        setpoint_umi = setpoint_umi - 1;
      }
      else {
        setpoint_umi = setpoint_umi - 0.1;
      }
    }
    if (seletor == 2) {
      if (millis() - salva_click < 500) {
        incrementador_salva++;
        if (incrementador_salva > 10) {
          altera_rapido = true;
        }
      }
      if (millis() - salva_click > 500) {
        incrementador_salva = 0;
        altera_rapido = false;
      }
      salva_click = millis();
      if (altera_rapido == true) {
        setpoint_co2 = setpoint_co2 - 1;
      }
      else {
        setpoint_co2 = setpoint_co2 - 0.1;
      }
    }

  }

  if (key == '8') { //tecla que aumenta o setpoint
    if (seletor == 0) {
      if (millis() - salva_click < 500) {
        incrementador_salva++;
        if (incrementador_salva > 10) {
          altera_rapido = true;
        }
      }
      if (millis() - salva_click > 500) {
        incrementador_salva = 0;
        altera_rapido = false;
      }
      salva_click = millis();
      if (altera_rapido == true) {
        setpoint_temp = setpoint_temp + 1;
      }
      else {
        setpoint_temp = setpoint_temp + 0.1;
      }
    }
    if (seletor == 1) {
      if (millis() - salva_click < 500) {
        incrementador_salva++;
        if (incrementador_salva > 10) {
          altera_rapido = true;
        }
      }
      if (millis() - salva_click > 500) {
        incrementador_salva = 0;
        altera_rapido = false;
      }
      salva_click = millis();
      if (altera_rapido == true) {
        setpoint_umi = setpoint_umi + 1;
      }
      else {
        setpoint_umi = setpoint_umi + 0.1;
      }
    }
    if (seletor == 2) {
      if (millis() - salva_click < 500) {
        incrementador_salva++;
        if (incrementador_salva > 10) {
          altera_rapido = true;
        }
      }
      if (millis() - salva_click > 500) {
        incrementador_salva = 0;
        altera_rapido = false;
      }
      salva_click = millis();
      if (altera_rapido == true) {
        setpoint_co2 = setpoint_co2 + 1;
      }
      else {
        setpoint_co2 = setpoint_co2 + 0.1;
      }
    }

  }

  if (key == '9' && tela_setpoint == true) { //botao ira enviar os novos setpoints para a EEPROM
    aux_int = (setpoint_temp * 100) + 1;
    temp_int = aux_int / 100;
    aux_dec = (aux_int - (temp_int * 100));
    temp_dec = aux_dec / 10;

    aux_int = (setpoint_umi * 100) + 1;
    umi_int = aux_int / 100;
    aux_dec = (aux_int - (umi_int * 100));
    umi_dec = aux_dec / 10;

    aux_int = (setpoint_co2 * 100) + 1;
    co2_int = aux_int / 100;
    aux_dec = (aux_int - (co2_int * 100));
    co2_dec = aux_dec / 10;

    EEPROM.write(0, temp_int);
    EEPROM.write(1, temp_dec);
    EEPROM.write(2, umi_int);
    EEPROM.write(3, umi_dec);
    EEPROM.write(4, co2_int);
    EEPROM.write(5, co2_dec);
    EEPROM.write(6, temp_int);
    EEPROM.write(7, temp_dec);
    EEPROM.write(8, umi_int);
    EEPROM.write(9, umi_dec);
    EEPROM.write(10, co2_int);
    EEPROM.write(11, co2_dec);
    EEPROM.write(12, temp_int);
    EEPROM.write(13, temp_dec);
    EEPROM.write(14, umi_int);
    EEPROM.write(15, umi_dec);
    EEPROM.write(16, co2_int);
    EEPROM.write(17, co2_dec);
    //Serial.println(temp_int);
    //Serial.println(temp_dec);
    //Serial.println(umi_int);
    //Serial.println(umi_dec);
    //Serial.println(co2_int);
    //Serial.println(co2_dec);
    EEPROM.commit();
    salva_setpoints_enviados = millis();
    tela_setpoints_enviados = true;
    setpoint_temp_int = setpoint_temp * 100;
    setpoint_umi_int = setpoint_umi * 100;
    setpoint_co2_int = setpoint_co2 * 100;
    //    setpoint_temp_int = 10;
    //    setpoint_umi_int = 11;
    //    setpoint_co2_int = 9;
    setpoint_temp_int++;
    setpoint_umi_int++;
    setpoint_co2_int++;
    sprintf(bufferfinal, "%ld,%ld,%ld,", setpoint_temp_int, setpoint_umi_int, setpoint_co2_int);
    Serial2.write(bufferfinal);
    Serial2.write('\n');
  }


  if (millis() - salva_setpoints_enviados > 5000) {
    tela_setpoints_enviados = false;
  }

  if (millis() - att_tela > 300) {
    if (tela_setpoints_enviados == true) {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("SETPOINTS SALVOS");
      lcd.setCursor(0, 2);
      lcd.print("E VERIFICADOS");
    }
    else {
      if (tela_setpoint == true) {
        if (seletor == 0) {
          lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("SETPOINTS");
          lcd.setCursor(0, 1);
          lcd.print("TEMPERATURA:");
          lcd.setCursor(13, 1);
          lcd.print(setpoint_temp);
          //seletor
          lcd.setCursor(19, 1); //seletor na posicao da temperatura
          lcd.print('*');
          lcd.setCursor(0, 2);
          lcd.print("UMIDADE:");
          lcd.setCursor(9, 2);
          lcd.print(setpoint_umi);
          lcd.setCursor(0, 3);
          lcd.print("CO2:");
          lcd.setCursor(5, 3);
          lcd.print(setpoint_co2);
        }
        if (seletor == 1) {
          lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("SETPOINTS");
          lcd.setCursor(0, 1);
          lcd.print("TEMPERATURA:");
          lcd.setCursor(13, 1);
          lcd.print(setpoint_temp);
          lcd.setCursor(0, 2);
          lcd.print("UMIDADE:");
          lcd.setCursor(9, 2);
          lcd.print(setpoint_umi);
          //seletor
          lcd.setCursor(19, 2); //seletor na posicao da umidade
          lcd.print('*');
          lcd.setCursor(0, 3);
          lcd.print("CO2:");
          lcd.setCursor(5, 3);
          lcd.print(setpoint_co2);
        }

        if (seletor == 2) {
          lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("SETPOINTS");
          lcd.setCursor(0, 1);
          lcd.print("TEMPERATURA:");
          lcd.setCursor(13, 1);
          lcd.print(setpoint_temp);
          lcd.setCursor(0, 2);
          lcd.print("UMIDADE:");
          lcd.setCursor(9, 2);
          lcd.print(setpoint_umi);
          lcd.setCursor(0, 3);
          lcd.print("CO2:");
          lcd.setCursor(5, 3);
          lcd.print(setpoint_co2);

          //seletor
          lcd.setCursor(19, 3); //seletor na posicao do co2
          lcd.print('*');
        }
      }
      if (tela_setpoint == false && temp <= 100 && umi <= 100 && co2 <= 100) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(hora);
        lcd.setCursor(2, 0);
        lcd.print(':');
        lcd.setCursor(3, 0);
        lcd.print(minuto);
        lcd.setCursor(5, 0);
        lcd.print(':');
        lcd.print(segundo);

        lcd.setCursor(16, 0);
        lcd.print(ano);
        lcd.setCursor(15, 0);
        lcd.print('/');
        lcd.setCursor(13, 0);
        lcd.print(mes);
        lcd.setCursor(12, 0);
        lcd.print('/');
        lcd.setCursor(10, 0);
        lcd.print(dia);

        lcd.setCursor(0, 1);
        lcd.print("TEMP:");
        lcd.setCursor(7, 1);
        lcd.print(temp);

        lcd.setCursor(0, 2);
        lcd.print("UMI:");
        lcd.setCursor(7, 2);
        lcd.print(umi);

        lcd.setCursor(0, 3);
        lcd.print("CO2:");
        lcd.setCursor(7, 3);
        lcd.print(co2);
      }
      att_tela = millis();
    }
  }






  //salva no cartao sd
  if (millis() - salva_sd > 300000) {
    sprintf(salvamento, "%ld,%ld,%ld,%d:%d:%d,%d/%d/%d,", leitura_temp, leitura_umi, leitura_co2, hora, minuto, segundo, dia, mes, ano);
    //appendFile(SD,"/d0.txt",mensagem);
    appendFile(SD, "/d0.txt", salvamento);
    appendFile(SD, "/d0.txt", "\r\n");

    salva_sd = millis();
  }

  if (millis() - tempo_envia_reset > 10000) {
    Serial2.write('r');
    Serial2.write('\n');
    tempo_envia_reset = millis();
  }

  if (millis() - envio_forcado_setpoint > 60000 || passe_envio == true) {
    setpoint_temp_int = setpoint_temp * 100;
    setpoint_umi_int = setpoint_umi * 100;
    setpoint_co2_int = setpoint_co2 * 100;
    setpoint_temp_int++;
    setpoint_umi_int++;
    setpoint_co2_int++;
    sprintf(bufferfinal, "%ld,%ld,%ld,", setpoint_temp_int, setpoint_umi_int, setpoint_co2_int);
    Serial2.write(bufferfinal);
    Serial2.write('\n');
    //appendFile(SD,"/d0.txt","setpoints reenviados");
    //appendFile(SD,"/d0.txt","\r\n");
    envio_forcado_setpoint = millis();
    passe_envio = false;
  }

}

//funcoes do cartao sd
void listDir(fs::FS &fs, const
             char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}
