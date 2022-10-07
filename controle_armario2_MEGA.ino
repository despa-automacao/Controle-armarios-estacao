//sem a leitura do fim de curso
//3 segundos pra sair do setup
//codigo que já comeca por padrao com o controle das ativacoes, o stop tem que ser ativado manualmente
//sem a eeprom por completo

#include <EEPROM.h>

//declaracao de pinos que são usados nos controles
#define R3_NADA 11
#define AT 10
#define CO2 9
#define EXAUSTAO 8
#define AQUECEDOR 7
#define UMIDIFICADOR 5                       
#define COOLER_AQUECEDOR 6
#define R1_NADA 27
#define R7_NADA 28
#define BOBINA_T1 29
#define BOMBA_EXAUSTAO 31
#define R10_RAD_INTERNO 32
#define COOLER_UMIDIFICADOR 33
#define R12_NADA 34
#define LED_PORTA_ABERTA 35
#define LED_EXAUSTAO 4
#define LED_ATUANDO 3
#define LED_VAZANDO 2
#define LED_LIGADO 22
#define LED_EM_OPERACAO 23
#define LED_PAUSADO 24
#define LED_STOP 25
#define RESFRIADOR 26 
#define FIM_CURSO 40
#define CO2_EXT1 36
#define CO2_EXT2 37
#define DESUMIDIFICADOR1 38
#define DESUMIDIFICADOR2 39
#define PULSO 29
#define NANO 12

//pinos sensor externo CO2
const int pino_sensor = 37;
const int pino_analog = A0;

//salva millis
unsigned long salvaUart = 0;
unsigned long salva_setpoint = 0;
unsigned long tempo_resfriador = 0;
unsigned long tempo_resistencia = 0;
unsigned long tempo_segura_umidificador = 0;
unsigned long tempo_umidificador = 0;
unsigned long tempo_segura_co2 = 0;
unsigned long tempo_co2 = 0;
unsigned long tempo_pulso = 0;
unsigned long tempo_reset_esp32 = 0;
unsigned long tempo_reset_nano = 0;
unsigned long segura_reset_leitura = 0;
unsigned long segura_ativacoes = 0;
unsigned long sinal_mega_ligado = 1000;
unsigned long salva_reset_nano = 0;
unsigned long tempo_desumidificador = 0;
unsigned long verifica_vazao = 0;

//variaveis para salvar as leituras da maquina
int posicao = 0;
char mensagem[30] = "0000,0000,0000,0000,000000,";

//variaveis para salvar os setpoints
char mensagem_setpoint[18] = "0000,0000,0000," ;
int posicao_setpoint;

//variaveis para armazenar os setpoints
long setpoint_temp, setpoint_umi, setpoint_co2;
float setpoint_temp_float,setpoint_umi_float,setpoint_co2_float;

//variaveis para armazenar as leituras
long leitura_temp,leitura_umi,leitura_co2;
long leitura_temp2,leitura_umi2;
float leitura_temp_float,leitura_umi_float,leitura_co2_float;
float leitura_temp_float2,leitura_umi_float2;

//variavel para tirar o "ruido"
char esvazia;

//variaveis para salvar o setpoint na eeprom
int temp_int,umi_int,co2_int,temp_int1,umi_int1,co2_int1,temp_int2,umi_int2,co2_int2;
float temp_dec_aux,umi_dec_aux,co2_dec_aux;
int temp_dec,umi_dec,co2_dec,temp_dec1,umi_dec1,co2_dec1,temp_dec2,umi_dec2,co2_dec2;

//variaveis de controle(que vem do painel)
bool operar = true;
bool ligar_exaustao = false;
bool exaustao = false;
bool pause = false;

//variaveis de controle do co2
int estadoCo2 = 0;
int tempo_de_injecao_co2;
int tempo_de_homogenizacao_do_co2;

//variaveis de controle do resfriador
int estadoResfriador = 0;

//variaveis de controle do aquecedor
int estadoResistencias = 0;

//varivaceis do controle da umidade com umidificador
int estadoUmidificador = 0;
bool permissao_umidade = false;
long tempo_umidificador_ligado;
long tempo_umidificador_desligado;

//variaveis do controle da umidade com o desumidificador
bool controle_desumidificacao = false;
int estadoDesumidificador = 0;

//variaveis para iniciacao do armario, lembrando o ultimo estado de quando foi desligado
bool estado_armario,estado_armario1,estado_armario2;

//variaveis de reset por pulso
bool aux_pulso = false;
bool permissao_pulso = true;
int acumulador_reset = 0;

//variaveis de backup de leitura, para tirar as leituras invalidas/erradas
//long bk_leitura_temp,bk_leitura_umi,bk_leitura_co2;

//incrementador de reset por leitura 0 do sensor
int incrementador_leitura_0 = 0;

//variavel para controle de permissao do reset nano
bool permissao_if_reset = false;

//variaveis de backup de valores
float backup_temp_float;
float backup_umi_float;
float backup_temp_float2;
float backup_umi_float2;
float backup_co2_float;
long backup_temp;
long backup_umi;
long backup_temp2;
long backup_umi2;
long backup_co2;

//variaveis para controle dos leds do setpoint
bool setpoint_int = false;

//variaveis para a medicao externa de co2
int leitura_co2_ext;
bool bloqueio_injecao_co2 = false;

//variaveis auxiliares do sistema de ativacao imediata do sistema de refrigeracao
bool bloqueio_ativacao_imediata_refrigeracao = true;

//variaveis auxiliares do bloqueio do sistema de refrigeracao se o setpoint de temp for maior que 35 graus
bool trava_refrigeracao = false;


void setup() {
  //incia as seriais
  Serial.begin(9600);   //usada para a comunicacao do monitor serial do computador
  Serial1.begin(9600);  //usada para a comunicacao com o ESP32 (envio dos dados de leituras e sinal de controlador ligado)
  Serial3.begin(9600);  //usada para a comunicacao com o ESP32 (usado para receber os dados que o ESP32 envia)
  Serial2.begin(9600);  //usada para a cominucacao com o arduino NANO

  //define os tipos de pinos do arduino mega
  pinMode(COOLER_AQUECEDOR,OUTPUT);
  pinMode(COOLER_UMIDIFICADOR,OUTPUT);
  pinMode(LED_LIGADO,OUTPUT);
  pinMode(FIM_CURSO,INPUT);
  pinMode(LED_PORTA_ABERTA,OUTPUT);
  pinMode(LED_EM_OPERACAO,OUTPUT);
  pinMode(EXAUSTAO,OUTPUT);
  pinMode(AT,OUTPUT);
  pinMode(BOMBA_EXAUSTAO,OUTPUT);
  pinMode(LED_EXAUSTAO,OUTPUT);
  pinMode(RESFRIADOR,OUTPUT);
  pinMode(R10_RAD_INTERNO,OUTPUT);
  pinMode(UMIDIFICADOR,OUTPUT);
  pinMode(DESUMIDIFICADOR1,OUTPUT);
  pinMode(DESUMIDIFICADOR2,OUTPUT);
  pinMode(AQUECEDOR,OUTPUT);
  pinMode(LED_PAUSADO,OUTPUT);
  pinMode(LED_ATUANDO,OUTPUT);
  pinMode(CO2,OUTPUT);
  pinMode(LED_STOP,OUTPUT);
  pinMode(PULSO,OUTPUT);
  pinMode(NANO,OUTPUT);
  pinMode(LED_VAZANDO,OUTPUT);
  
  //iniciacao do pinos do sensor externo de co2
  pinMode(pino_sensor,INPUT);
  pinMode(pino_analog,INPUT);
  
  //ativacoes que devem comecar desligadas
  digitalWrite(EXAUSTAO,HIGH);
  digitalWrite(AT,HIGH);
  digitalWrite(BOMBA_EXAUSTAO,HIGH);
  digitalWrite(LED_EXAUSTAO,HIGH);
  digitalWrite(RESFRIADOR,HIGH);
  digitalWrite(UMIDIFICADOR,HIGH);
  digitalWrite(DESUMIDIFICADOR1,HIGH),
  digitalWrite(DESUMIDIFICADOR2,HIGH),
  digitalWrite(AQUECEDOR,HIGH);
  digitalWrite(LED_PAUSADO,HIGH);
  digitalWrite(LED_ATUANDO,LOW);
  digitalWrite(CO2,HIGH);
  digitalWrite(LED_STOP,HIGH);
  digitalWrite(PULSO,HIGH);
  digitalWrite(LED_EM_OPERACAO,HIGH);
  digitalWrite(LED_ATUANDO,HIGH);
  digitalWrite(LED_VAZANDO,HIGH);
  
  //ativacao paleativa REMOVER DEPOIS QUE HABILITAR O FIM DE CURSO
  //DEPOIS QUE O FIM DE CURSO SER CONSERTADO, DESCOMENTAR ESSA PARTE 
  digitalWrite(LED_PORTA_ABERTA,HIGH);
  
  //ativacoes que devem comecar ligadas
  //digitalWrite(LED_PORTA_ABERTA,LOW);
  digitalWrite(NANO,LOW);
  delay(1000);
  digitalWrite(NANO,HIGH);
  

  
  //liga o led "ligado"
  digitalWrite(LED_LIGADO,LOW);
 

  //define o estado inicial da maquina como EM OPERACAO, desde o inicio
  //para caso a maquina reinicie, nao seja necessario algume vir e apertar o botao verde para voltar a controlar os setpoins
  operar = true;
  delay(2000);  
}

void loop() {

    //parte do codigo que recebe do arduino NANO as leituras que os sensores fizeram
    //toda vez que o while detecta que tem algo no buffer da serial, um caracter é lido e armazenado no array de recebidos
    //caso o caractere lido seja um \n os dados da mensagem sao processados, indicando que o final da mensagem foi lido/recebido
    //no final do processamento dos dados, a mensagem inteira lida é enviada para o ESP32, para os dados serem jogados no display (sao os SERIAL1.WRITE no final)
    //if(millis() - salvaUart > 2000){
      while(Serial2.available()>0){
      char recebido = Serial2.read();
      if(recebido != ',' && recebido != '0' && recebido != '1' && recebido != '2' && recebido != '3' && recebido != '4' && recebido != '5' && recebido != '6' && recebido != '7' && recebido != '8' && recebido != '9' && recebido != '\n' && recebido != '\0'){
        posicao = 0;
        }
        else{    
      if(recebido != '\n'){
      //if(recebido != '\n' && (posicao < 19 -1)){
        mensagem[posicao] = recebido;
        posicao++; 
      }
      else{
        mensagem[posicao] = '\0';
         
        
        
        posicao = 0;
        sscanf(mensagem, "%ld,%ld,%ld,%ld,%ld,",&leitura_temp,&leitura_umi,&leitura_temp2,&leitura_umi2,&leitura_co2);
        tempo_reset_nano = millis();


            
        if((leitura_temp == 0) || (leitura_umi == 0) || (leitura_temp2 == 0) || (leitura_umi2 == 0) || (leitura_co2 == 0) || (leitura_temp < 0) || (leitura_umi < 0) || (leitura_temp2 < 0) || (leitura_umi2 < 0) || (leitura_temp > 10000) || (leitura_umi > 10000) || (leitura_temp2 > 10000) || (leitura_umi2 > 10000)){
            incrementador_leitura_0++;
            leitura_temp = backup_temp;
            leitura_umi = backup_umi;
            leitura_temp2 = backup_temp2;
            leitura_umi2 = backup_umi2;
            leitura_co2 = backup_co2;
          }
        else{
          incrementador_leitura_0 = 0;
            //só envia pro display se as leituras forem "corretas" (sem erro)
 
            Serial.println(mensagem);      
                    //transforma a leitura em float
                    
            leitura_temp_float = float(leitura_temp)/100.00;
            leitura_umi_float = float(leitura_umi)/100.00;
            leitura_temp_float2 = float(leitura_temp2)/100.00;
            leitura_umi_float2 = float(leitura_umi2)/100.00;
            leitura_co2_float = float(leitura_co2)/10000.00;

            backup_temp = leitura_temp;
            backup_umi = leitura_umi;
            backup_temp2 = leitura_temp2;
            backup_umi2 = leitura_umi2;
            backup_co2 = leitura_co2;
            
            Serial.println(mensagem);
            Serial.println(leitura_temp_float);
            Serial.println(leitura_umi_float);
            Serial.println(leitura_temp_float2);
            Serial.println(leitura_umi_float2);
            Serial.println(leitura_co2_float);
            Serial.println();
            leitura_temp_float = (leitura_temp_float + leitura_temp_float2)/2;
            leitura_umi_float = (leitura_umi_float + leitura_umi_float2)/2;
//            Serial.println(leitura_temp_float);
//            Serial.println(leitura_umi_float);   
//            leitura_temp = (leitura_temp + leitura_temp2)/2;
//            leitura_umi = (leitura_umi + leitura_umi2)/2;
            //sprintf(mensagem,"%ld,%ld,%ld,",leitura_temp,leitura_umi,leitura_co2);
            Serial1.write(mensagem);
            Serial1.write('\n'); 
          }
      
        if(incrementador_leitura_0>10){    
          while(1){
            Serial.println("entrou aqui");
            digitalWrite(PULSO,LOW);
            digitalWrite(NANO,LOW);
            }
          }
        

        }
       }
      }
    //salvaUart = millis();  
    //}
    
 
       //parte do codigo que recebe do ESP32 os comandos do teclado, e os SETPOINTS de operacao
       //o arduino MEGA nao salva os setpoints na EEPROM
       //toda vez que o while detecta que tem algo no buffer da serial, um caracter é lido e armazenado no array de recebidos
       //caso o caractere lido seja um \n os dados da mensagem sao processados, indicando que o final da mensagem foi lido/recebido
       //caso o caractere recebido seja um 'a': o estado da maquina fica como operar = true
       //caso o caractere recebido seja um 'b': o estado da maquina fica como pause = true
       //caso o caractere recebido seja um 'c': o estado da maquina fica como operar = false
       //caso o caractere recebido seja um 'd': o estado da maquina fica como exaustao = true
       //caso o caractere recebido seja um 'r': o verificador do ESP32 ter travado é resetado, o ESP32 envia esse caractere de tempos em tempos para mostrar pra o MEGA que ele ainda esta funcionando e o sistema nao precisa ser resetado
       //caso os caracteres recebidos nao sejam letras, e sim numeros: o que esta sendo recebido são os valores de setpoint
          //os valores de setpoint sao interpretados da mesma forma que os caracteres
             //toda vez que o while detecta que tem algo no buffer da serial, um caracter é lido e armazenado no array de recebidos
             //caso o caractere lido seja um \n os dados da mensagem sao processados, indicando que o final da mensagem foi lido/recebido
          //os valores de setpoints sao reenviados de tempo em tempo
       while(Serial3.available()>0){
        Serial.println("recebeu do esp32");
        char recebido_setpoint = Serial3.read();
        if(recebido_setpoint != ',' && recebido_setpoint != '0' && recebido_setpoint != '1' && recebido_setpoint != '2' && recebido_setpoint != '3' && recebido_setpoint != '4' && recebido_setpoint != '5' && recebido_setpoint != '6' && recebido_setpoint != '7' && recebido_setpoint != '8' && recebido_setpoint != '9' && recebido_setpoint != '\n' && recebido_setpoint != '\0' ){
        Serial.print("recebido do display: ");
        Serial.println(recebido_setpoint);
        
        if(recebido_setpoint == 'a'){
          //faz algo se receber a
          operar = true;
//          EEPROM.write(30,1);   
//          EEPROM.write(31,1);  
//          EEPROM.write(32,1);  
          Serial.println("recebeu a operar true");
          pause = false;
          exaustao = false;
          posicao_setpoint = 0;
          estadoResistencias = 0;
          estadoResfriador = 0;
          estadoUmidificador = 0;
          
          }
        if(recebido_setpoint == 'b'){
          Serial.println("recebeu b pause");
          //faz algo se receber b
          pause = true;
          posicao_setpoint = 0; 
          digitalWrite(LED_ATUANDO,HIGH);
          }
        if(recebido_setpoint == 'c'){
          Serial.println("recebeu c operar false");
          //faz algo se receber c
          operar = false;
//          EEPROM.write(30,0);   
//          EEPROM.write(31,0);  
//          EEPROM.write(32,0); 
//          Serial.println("operar false");
          posicao_setpoint = 0;
          }
        if(recebido_setpoint == 'd'){
          Serial.println("recebeu d exaustao");
          //faz algo se receber d
          exaustao = true;
          posicao_setpoint = 0;
          
          }
        if(recebido_setpoint == 'r'){
          //codigo do reset enviado pelo esp32
          tempo_reset_esp32 = millis();
          Serial.println("resetou o tempo do esp32");
//          Serial.println("limpou o tempo");
          posicao_setpoint = 0; 
          }
        posicao_setpoint = 0;
        }
        else{
      if(recebido_setpoint != '\n'){
        mensagem_setpoint[posicao_setpoint] = recebido_setpoint;
        posicao_setpoint++;
      }
      else{
        mensagem_setpoint[posicao_setpoint] = '\0';
        //Serial.println(mensagem_setpoint);
        sscanf(mensagem_setpoint, "%ld,%ld,%ld,",&setpoint_temp,&setpoint_umi,&setpoint_co2);
        temp_int = setpoint_temp / 100;
        temp_dec_aux = setpoint_temp-(temp_int*100);
        temp_dec = temp_dec_aux/10;
        setpoint_temp_float = temp_int + (temp_dec*0.1);
//        EEPROM.write(0,temp_int);
//        EEPROM.write(1,temp_dec);
//        EEPROM.write(6,temp_int);
//        EEPROM.write(7,temp_dec);
//        EEPROM.write(12,temp_int);
//        EEPROM.write(13,temp_dec);    
            
        umi_int = setpoint_umi / 100;
        umi_dec_aux = (setpoint_umi-(umi_int*100));
        umi_dec = umi_dec_aux/10;
        setpoint_umi_float = umi_int + (umi_dec*0.1);
//        EEPROM.write(2,umi_int);
//        EEPROM.write(3,umi_dec);
//        EEPROM.write(8,umi_int);
//        EEPROM.write(9,umi_dec);
//        EEPROM.write(14,umi_int);
//        EEPROM.write(15,umi_dec);     
                   
        co2_int = setpoint_co2 / 100;
        co2_dec_aux = (setpoint_co2-(co2_int*100));
        co2_dec = co2_dec_aux/10;
        setpoint_co2_float = co2_int + (co2_dec*0.1);        
//        EEPROM.write(4,co2_int);
//        EEPROM.write(5,co2_dec);
//        EEPROM.write(10,co2_int);
//        EEPROM.write(11,co2_dec);        
//        EEPROM.write(16,co2_int);
//        EEPROM.write(17,co2_dec);        
        posicao_setpoint = 0;
        }
      }
    }

//    if(millis() - salva_setpoint > 2000){
//    sscanf(mensagem_setpoint, "%ld,%ld,%ld,",&setpoint_temp,&setpoint_umi,&setpoint_co2);
////    Serial.println(mensagem_setpoint);
////    Serial.println(); 
////    Serial.println(setpoint_temp_float);
////    Serial.println(setpoint_umi_float);
////    Serial.println(setpoint_co2_float);    
//    temp_int = setpoint_temp / 100;
//    temp_dec_aux = setpoint_temp-(temp_int*100);
//    temp_dec = temp_dec_aux/10;
////    EEPROM.write(0,temp_int);
////    EEPROM.write(1,temp_dec);
////    EEPROM.write(6,temp_int);
////    EEPROM.write(7,temp_dec);
////    EEPROM.write(12,temp_int);
////    EEPROM.write(13,temp_dec); 
//        
//    umi_int = setpoint_umi / 100;
//    umi_dec_aux = (setpoint_umi-(umi_int*100));
//    umi_dec = umi_dec_aux/10;
////    EEPROM.write(2,umi_int);
////    EEPROM.write(3,umi_dec);
////    EEPROM.write(8,umi_int);
////    EEPROM.write(9,umi_dec);
////    EEPROM.write(14,umi_int);
////    EEPROM.write(15,umi_dec);  
//        
//    co2_int = setpoint_co2 / 100;
//    co2_dec_aux = (setpoint_co2-(co2_int*100));
//    co2_dec = co2_dec_aux/10;           
////    EEPROM.write(4,co2_int);
////    EEPROM.write(5,co2_dec);
////    EEPROM.write(10,co2_int);
////    EEPROM.write(11,co2_dec);        
////    EEPROM.write(16,co2_int);
////    EEPROM.write(17,co2_dec);   
//    salva_setpoint = millis();
//    }

    //verificacao do fim de curso
//    if(digitalRead(FIM_CURSO) == LOW){
//      //Serial.println("ligou led porta aberta");
//      pinMode(LED_PORTA_ABERTA,LOW);
//      }
//    if(digitalRead(FIM_CURSO) == HIGH){
//      pinMode(LED_PORTA_ABERTA,HIGH);
//      operar = false;
//      EEPROM.write(18,0);   
//      EEPROM.write(19,0);  
//      EEPROM.write(20,0);   
//      }      
//      else{
//        pinMode(LED_PORTA_ABERTA,LOW);
//        }


    //controle dos leds do painel
    if(operar == true){
      //desliga o led vermelho STOP
      digitalWrite(LED_STOP,HIGH);
      
      //desliga o led pausado
      if(setpoint_int == true){
        digitalWrite(LED_PAUSADO,LOW);
        digitalWrite(LED_ATUANDO,HIGH);  
        }
        else{
          digitalWrite(LED_PAUSADO,HIGH);  
          }
      
      //controle dos leds do painel caso o pausado esteja ativo
      //e set do estado das ativacoes para o estado 2, onde as ativacoes esperam o tempo para serem rearmadas
          //como o estado é sempre setado para 2, as ativacoes nunca rearmam e ficam "pausadas"
      if(pause == true){
        //desliga o led atuando
        digitalWrite(LED_ATUANDO,HIGH);
        
        //ativa o led pausado
        digitalWrite(LED_PAUSADO,LOW);
        
        //desliga as ativações mas nao os coolers
          //liga os coolers
        digitalWrite(COOLER_AQUECEDOR,LOW);
        digitalWrite(COOLER_UMIDIFICADOR,LOW);
        digitalWrite(R10_RAD_INTERNO,LOW);
        digitalWrite(LED_EM_OPERACAO,LOW);

        //desliga as ativações
        digitalWrite(LED_EM_OPERACAO,HIGH);
        //desliga aquecedor
        digitalWrite(AQUECEDOR, HIGH);
        estadoResistencias = 2;                                                                               //controle do tempo e de ativação e desativação das resistencias
        //desliga resfriador
        digitalWrite(RESFRIADOR, HIGH);
        estadoResfriador = 2;                                                                                 //controle do tempo e de ativação e desativação das resistencias
        //desliga umidificador
        digitalWrite(UMIDIFICADOR, HIGH);
        estadoUmidificador = 2;
        //desliga desumidificador
        digitalWrite(DESUMIDIFICADOR1,HIGH);
        digitalWrite(DESUMIDIFICADOR2,HIGH);  
        //desliga co2
        digitalWrite(CO2, HIGH);
        estadoCo2 = 2;         
        }
        //liga os coolers
      digitalWrite(COOLER_AQUECEDOR,LOW);
      digitalWrite(COOLER_UMIDIFICADOR,LOW);
      digitalWrite(R10_RAD_INTERNO,LOW);
      digitalWrite(LED_EM_OPERACAO,LOW);
      
      //adicionar o controle dos ativadores aqui########################################################
      //Serial.println("entrou na ativacao");

    //inicio do controle das ativacoes
    //o programa espera 40 segundos para comecar a fazer o controle da temperatura, umidade de co2
    if(millis() - segura_ativacoes> 40000){

      //checa a variavel de permissao de ativacao do sistema de umidade
      //o programa so comeca a controlar a umidade se o setpoint da temperatura for menor que a leitura 
      //uma vez que a permissao vira TRUE, ela nunca mais volta pra FALSE
      if(leitura_temp_float > setpoint_temp_float){
        permissao_umidade = true;
        }  

      //se o setpoint da temperatura é maior que 35 graus, o sistema de refrigeracao nao liga
      //controle da trava da ativacao da refrigeracao
      if(setpoint_temp_float >= 35){
        trava_refrigeracao = true;
        }
        else{
          trava_refrigeracao = false;
          }

         
      //controle da ativacao imediata da refrigeracao
      if((leitura_temp_float > setpoint_temp_float+0.85) && bloqueio_ativacao_imediata_refrigeracao == false){
        estadoResfriador = 0;  
        bloqueio_ativacao_imediata_refrigeracao = true;
        }


      //INICIO DO CONTROLE DO RESFRIADOR
      if(leitura_temp_float > setpoint_temp_float+0.85){
        //faz o controle do resfriador
          if(estadoResfriador == 0){  
  //          Serial.println("ligou o resfriador");//controle do tempo e de ativação e desativação das resistencias
            if(trava_refrigeracao == false){ 
              digitalWrite(RESFRIADOR,LOW);
            }
            estadoResfriador = 1;                                                                                 //controle do tempo e de ativação e desativação das resistencias
            tempo_resfriador = millis();                                                                          //controle do tempo e de ativação e desativação das resistencias 
          }       
          if ((millis() - tempo_resfriador) > 25000 && estadoResfriador == 1){ //15000              //controle do tempo e de ativação e desativação das resistencias
            digitalWrite(RESFRIADOR, HIGH);
            estadoResfriador = 2;                                                                                 //controle do tempo e de ativação e desativação das resistencias
            tempo_resfriador = millis();                                                                          //controle do tempo e de ativação e desativação das resistencias
          }
          if ((millis() - tempo_resfriador) > 180000 && estadoResfriador == 2){//150000            //controle do tempo de ativação e desativação das resistencias
            estadoResfriador = 0;                                                                                //controle do tempo de ativação e desativação das resistencias
          }
        }
        else{
          if (estadoResfriador == 1){               //controle do tempo e de ativação e desativação das resistencias
            digitalWrite(RESFRIADOR, HIGH);
            estadoResfriador = 2;                                                                                 //controle do tempo e de ativação e desativação das resistencias
            tempo_resfriador = millis();                                                                          //controle do tempo e de ativação e desativação das resistencias
          }
          if ((millis() - tempo_resfriador) > 150000 && estadoResfriador == 2){            //controle do tempo de ativação e desativação das resistencias
            estadoResfriador = 0;                                                                                //controle do tempo de ativação e desativação das resistencias
          }          
          
          }
        
        if(leitura_temp_float < setpoint_temp_float){
          bloqueio_ativacao_imediata_refrigeracao = false;
                if (estadoResistencias == 0){                                //controle do tempo e de ativação e desativação das resistencias
                  digitalWrite(AQUECEDOR,LOW);
//                  Serial.println("ligou a resistencia");
                  estadoResistencias = 1;                                                                                     //controle do tempo e de ativação e desativação das resistencias
                  tempo_resistencia = millis();                                                                          //controle do tempo e de ativação e desativação das resistencias 
                }
                  
                if ((millis() - tempo_resistencia) > 5000 && estadoResistencias == 1){               //controle do tempo e de ativação e desativação das resistencias
                  digitalWrite(AQUECEDOR, HIGH);
                  estadoResistencias = 2;                                                                               //controle do tempo e de ativação e desativação das resistencias
                  tempo_resistencia = millis();                                                                          //controle do tempo e de ativação e desativação das resistencias
                }
                  
                if ((millis() - tempo_resistencia) > 15000 && estadoResistencias == 2){            //controle do tempo de ativação e desativação das resistencias
                  estadoResistencias = 0;                                                                                //controle do tempo de ativação e desativação das resistencias
                }       
              }
              else{
                if ((millis() - tempo_resistencia) > 5000 && estadoResistencias == 1){               //controle do tempo e de ativação e desativação das resistencias
                  digitalWrite(AQUECEDOR, HIGH);
                  estadoResistencias = 2;                                                                               //controle do tempo e de ativação e desativação das resistencias
                  tempo_resistencia = millis();                                                                          //controle do tempo e de ativação e desativação das resistencias
                }
                  
                if ((millis() - tempo_resistencia) > 15000 && estadoResistencias == 2){            //controle do tempo de ativação e desativação das resistencias
                  estadoResistencias = 0;                                                                                //controle do tempo de ativação e desativação das resistencias
                } 
            }
            
          //controle de desumidificação
//          if(leitura_umi_float > setpoint_umi_float+0.5){
//          digitalWrite(DESUMIDIFICADOR1,LOW);
//          digitalWrite(DESUMIDIFICADOR2,LOW);
//          }
//          else{
//            if(leitura_umi_float <= setpoint_umi_float){
//              digitalWrite(DESUMIDIFICADOR1,HIGH);
//              digitalWrite(DESUMIDIFICADOR2,HIGH);
//              }  
//            }

        //inicio do controle do 
        if(leitura_umi_float > setpoint_umi_float+0.5){
                                                 //controle do tempo e de ativação e desativação das resistencias
            digitalWrite(DESUMIDIFICADOR1,LOW);
            digitalWrite(DESUMIDIFICADOR2,LOW);
          }
          else{
            if(leitura_umi_float <= setpoint_umi_float){                                       //controle do tempo e de ativação e desativação das resistencias
            digitalWrite(DESUMIDIFICADOR1,HIGH);
            digitalWrite(DESUMIDIFICADOR2,HIGH);
              }
            }        
        if(leitura_umi_float+0.5 < setpoint_umi_float){                                      //controle do tempo e de ativação e desativação das resistencias
            digitalWrite(UMIDIFICADOR,LOW);
        }
        else{
            if(leitura_umi_float >= setpoint_umi_float){                                       //controle do tempo e de ativação e desativação das resistencias
              digitalWrite(UMIDIFICADOR, HIGH);  
            }
          }
        

        
      //################################################################################################
    if(bloqueio_injecao_co2 == false){
    if(leitura_co2_float < setpoint_co2_float){
      if(millis() - tempo_segura_co2 > 60000){
        verifica_estado_co2(setpoint_co2_float,leitura_co2_float);
        if (estadoCo2 == 0) { 
          //liga co2                                       
          digitalWrite(CO2,LOW);
          estadoCo2 = 1;     
                                                                                       
          tempo_co2 = millis();                                                                          
        }
        if ((millis() - tempo_co2) > tempo_de_injecao_co2 && estadoCo2 == 1) {              
          //desliga co2
          digitalWrite(CO2, HIGH);
          estadoCo2 = 2; 
                                                                                        
          tempo_co2 = millis();                                                                 
        }
        if ((millis() - tempo_co2) > tempo_de_homogenizacao_do_co2 && estadoCo2 == 2) {         
          estadoCo2 = 0;                                                                             
        }
      }      
    }
    else{
        if ((millis() - tempo_co2) > tempo_de_injecao_co2 && estadoCo2 == 1) {            
          //desliga co2
          digitalWrite(CO2, HIGH);
          estadoCo2 = 2;                                                                                 
          tempo_co2 = millis();                                                                          
        }
        if ((millis() - tempo_co2) > tempo_de_homogenizacao_do_co2 && estadoCo2 == 2) {           
          estadoCo2 = 0;                                                                               
        }
      }

      }
      else{
        if(bloqueio_injecao_co2 == true){
          if ((millis() - tempo_co2) > tempo_de_injecao_co2 && estadoCo2 == 1) {            
            //desliga co2
            digitalWrite(CO2, HIGH);
            estadoCo2 = 2;                                                                                 
            tempo_co2 = millis();                                                                          
          }
          if ((millis() - tempo_co2) > tempo_de_homogenizacao_do_co2 && estadoCo2 == 2) {           
            estadoCo2 = 0;                                                                               
          }          
          }
        }
    }
  }
    if(operar == false){
      
      //liga led vermelho stop
      digitalWrite(LED_STOP,LOW);
      //desliga o led pausado 
      digitalWrite(LED_PAUSADO,HIGH);
      //desliga o led atuando se nao foi exaustao
//      if(exaustao == true){             //???
//        digitalWrite(LED_ATUANDO,LOW);
//        }
//        else{
//        digitalWrite(LED_ATUANDO,HIGH);
//        }
      digitalWrite(LED_ATUANDO,HIGH); 
        //desliga os coolers
      digitalWrite(COOLER_AQUECEDOR,HIGH);
      digitalWrite(COOLER_UMIDIFICADOR,HIGH);
      digitalWrite(R10_RAD_INTERNO,HIGH);
      digitalWrite(LED_EM_OPERACAO,HIGH);

      
      digitalWrite(LED_EM_OPERACAO,HIGH);
      //desliga aquecedor
      digitalWrite(AQUECEDOR, HIGH);
      estadoResistencias = 2;                                                                               //controle do tempo e de ativação e desativação das resistencias
      //desliga resfriador
      digitalWrite(RESFRIADOR, HIGH);
      estadoResfriador = 2;                                                                                 //controle do tempo e de ativação e desativação das resistencias
      //desliga umidificador
      digitalWrite(UMIDIFICADOR, HIGH);
      estadoUmidificador = 2;
      //desliga desumidificador
      digitalWrite(DESUMIDIFICADOR1,HIGH);
      digitalWrite(DESUMIDIFICADOR2,HIGH);
      }

    if(exaustao == true){
      operar = false;
//      EEPROM.write(30,0);   
//      EEPROM.write(31,0);  
//      EEPROM.write(32,0);  
//      Serial.println("operar false");
      digitalWrite(LED_EXAUSTAO,LOW);   
      if(leitura_co2_float > 0.1){//trocar para 0.1
        digitalWrite(EXAUSTAO,LOW);
        digitalWrite(AT,LOW);
        digitalWrite(BOMBA_EXAUSTAO,LOW);
        Serial.println("ligou exaustao");
        }
      if(leitura_co2_float <= 0.1){//mudar depois pra 0.1
        digitalWrite(EXAUSTAO,HIGH);
        digitalWrite(AT,HIGH);
        digitalWrite(BOMBA_EXAUSTAO,HIGH);
        Serial.println("desligou exaustao");
        exaustao = false;
        }
      }
      if(exaustao == false){
        digitalWrite(EXAUSTAO,HIGH);
        digitalWrite(AT,HIGH);
        digitalWrite(BOMBA_EXAUSTAO,HIGH);
        digitalWrite(LED_EXAUSTAO,HIGH);
        }

  //verifica a ativacao do led azul ATUANDO e do led amarelo SETPOINT IN

//  if(setpoint_temp_float-2.0 < leitura_temp_float && setpoint_temp_float+2.0 > leitura_temp_float && setpoint_umi_float-5.0 < leitura_umi_float && setpoint_umi_float+5.0 > leitura_umi_float && leitura_co2_float+0.1 > setpoint_co2_float && operar == true){
//      //liga led do setpoint in
//      digitalWrite(LED_PAUSADO,LOW);
//      digitalWrite(LED_ATUANDO,HIGH);
//    }
//    else{
//      if(operar == true){
//        digitalWrite(LED_ATUANDO,LOW);
//        }
//        else{
//          digitalWrite(LED_ATUANDO,HIGH);
//          digitalWrite(LED_PAUSADO,HIGH);
//          }
//      }

  if((setpoint_temp_float-2.0 < leitura_temp_float) && (setpoint_temp_float+2.0 > leitura_temp_float) && operar == true){
    if((setpoint_umi_float-5.0 < leitura_umi_float) && (setpoint_umi_float+5.0 > leitura_umi_float) && operar == true){
      if((setpoint_co2_float-0.1 < leitura_co2_float) && (setpoint_co2_float+0.1 > leitura_co2_float) && operar == true){
//        digitalWrite(LED_ATUANDO,HIGH);
//        digitalWrite(LED_PAUSADO,LOW);
        setpoint_int = true;          
        }
        else{
          if(pause != true){
            digitalWrite(LED_ATUANDO,LOW);
            digitalWrite(LED_PAUSADO,HIGH);
            setpoint_int = false;     
            }
          }
      }
      else{
        if(pause != true){
          digitalWrite(LED_ATUANDO,LOW);
          digitalWrite(LED_PAUSADO,HIGH);
          setpoint_int = false;        
          }
        }
    }
    else{
      if(pause != true){
        digitalWrite(LED_ATUANDO,LOW);
        digitalWrite(LED_PAUSADO,HIGH);
        setpoint_int = false;
        }
      }


  //funcao de reset da maquina por pulso 
  if (((millis() - tempo_pulso) > 1500 && aux_pulso == false) || permissao_pulso == true){               
    aux_pulso = true;
    permissao_pulso = false;
    digitalWrite(PULSO,LOW);                                                                                 
    tempo_pulso = millis();                                                   
  }

  if ((millis() - tempo_pulso) > 13500 && aux_pulso == true){            
    digitalWrite(PULSO,HIGH);
    tempo_pulso = millis();
    aux_pulso = false;
  }

//verificacao da integridade de esp32 e ativacao do reset por pulso
//  if(millis() - tempo_reset_esp32 > 150000){
//    digitalWrite(NANO,LOW);
//    while(1){
//      Serial.println("RESET POR FALTA DE LEITURA DO ESP32");
//      digitalWrite(PULSO,LOW); 
//      
//      }
//    }

if(millis() - tempo_reset_nano > 60000 || permissao_if_reset == true){
  while(1){
    digitalWrite(NANO,LOW);
    digitalWrite(PULSO,LOW);
    Serial.println("reset nano");
    }
  
//  permissao_if_reset = true;
//  digitalWrite(NANO,LOW);
//  salva_reset_nano = millis();
//  if(millis() - salva_reset_nano > 5000){
//    digitalWrite(NANO,HIGH);
//    tempo_reset_nano = millis();
//    permissao_if_reset = false;
//    Serial.println("entrou no if para ligar denovo");
//    }
//  Serial.println("reset nano");
  }




if(millis() - sinal_mega_ligado > 10000){
    Serial1.write('a');
    Serial1.write('\n');
    sinal_mega_ligado = millis();
  }

//verifica se vazou co2 para o meio externo
//if(millis() - verifica_vazao > 1000){
//leitura_co2_ext = analogRead(pino_analog);
//leitura_co2_ext = map(leitura_co2_ext,0,1023,20,2000);
//Serial.print("CO2: ");
//Serial.println(leitura_co2_ext);
//if(leitura_co2_ext < 1000){
//  digitalWrite(LED_VAZANDO,HIGH);
//  bloqueio_injecao_co2 = false;
//  }
//if(leitura_co2_ext >= 1000){
//  digitalWrite(LED_VAZANDO,LOW);
//  bloqueio_injecao_co2 = true;  
//  }
//
//verifica_vazao = millis();
//}
}

//funcoes de curva de tempo de ativacao
void verifica_estado_umidificador(float set_point, float leitura){
  if((set_point - leitura) >= 10){
    tempo_umidificador_ligado =17500;
    tempo_umidificador_desligado = 5000;
    //return 0;
    }
  if((set_point - leitura) >= 6 && (set_point - leitura) < 10){
    tempo_umidificador_ligado = 15000;
    tempo_umidificador_desligado = 8000;
    //return 1;
    }
  if((set_point - leitura) >= 3 && (set_point - leitura) < 6){
    tempo_umidificador_ligado = 12500;
    tempo_umidificador_desligado = 11000;
    //return 2;
    }
  if((set_point - leitura) >= 1 && (set_point - leitura) < 3){
    tempo_umidificador_ligado = 10000;
    tempo_umidificador_desligado = 14000;
    //return 3;
    }
  if((set_point - leitura) < 1 ){
     tempo_umidificador_ligado = 7500;
     tempo_umidificador_desligado = 18000;
     //return 4;
     }  
}


void verifica_estado_co2(float set_point, float leitura){
  if((set_point - leitura) >= 2){
    tempo_de_injecao_co2 = 1500;
    tempo_de_homogenizacao_do_co2 = 8000;
    //return 0;
    }
  if((set_point - leitura) >= 1.1 && (set_point - leitura) < 2){
    tempo_de_injecao_co2 = 1200;
    tempo_de_homogenizacao_do_co2 = 10000;
    //return 1;
    }
  if((set_point - leitura) >= 0.51 && (set_point - leitura) < 1.1){
    tempo_de_injecao_co2 = 1000;
    tempo_de_homogenizacao_do_co2 = 15000;
    //return 2;
    }
  if((set_point - leitura) >= 0.11 && (set_point - leitura) < 0.51){
    tempo_de_injecao_co2 = 800;
    tempo_de_homogenizacao_do_co2 = 17000;
    //return 3;
    }
  if((set_point - leitura) < 0.11 ){
     tempo_de_injecao_co2 = 600;
     tempo_de_homogenizacao_do_co2 = 20000;
     //return 4;
     }  
}
