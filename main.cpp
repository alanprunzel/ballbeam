//Biblioteca do Servomotor
#include <Servo.h>
#include <Arduino.h>
#include <math.h>


/* INÍCIO - VARIÁVEIS PARA AMOSTRAGEM NO ENSAIOS COMPARATIVOS */
#define DEBUG 0                         // 0-modo normal    1-modo de amostragem continua

#ifdef DEBUG 
unsigned long previousMillis = 0;       // tempo da ultima amostragem realizada
const long interval = 10;               // intervalo de amostragem (milliseconds)
#endif
/* FIM - VARIÁVEIS PARA AMOSTRAGEM NO ENSAIOS COMPARATIVOS */


/* INÍCIO - VARIÁVEIS, CONSTANTES E PROTÓTIPOS GENÉRICOS */

//Cria um Objeto servoMotor
Servo bb_servo;

//Vetor para Filtro do Sinal de Saída da Planta - Sensor Softpot
int y_V[4] = {0,0,0,0};

//Vetor para Filtro do Sinal do Potenciômetro
int y_P[4] = {0,0,0,0};

//Variáveis Auxiliares Utilizadas na Linearização do Sensor SoftPot 
const double h = 549;    //549
const double p = 2036.5; //2036.5
const double k = 15;     //-37

//SetPoint
double set_point = 512;
/* FIM - VARIÁVEIS, CONSTANTES E PROTÓTIPOS GENÉRICOS */

/* INÍCIO - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO MANUAL */
//Protótipo do Procedimento do Módulo Manual
void manual_modulo();
/* FIM - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO MANUAL */

/* INÍCIO - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO FUZZY */
//Protótipo do Procedimento do Módulo Fuzzy
void fuzzy_modulo();
/* FIM - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO FUZZY */

/* INÍCIO - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO DEMO */
//Tempo de Amostragem em ms e s
const int demo_Ts = 10;
const double demo_Ts_segundos = 0.010;

//Constante N do Filtro do Termo Derivativo
const double demo_n = 6; // demo_n= 6

//Parâmetros do Controlador Paralelo
const double demo_kp = -1.042; // demo_kp= -1.042
const double demo_ki = -0.721; // demo_ki= -0.721
const double demo_kd = -0.586; // demo_kd= -0.586

const double demo_ti = demo_kp/demo_ki;
const double demo_td = demo_kd/demo_kp;

//Parâmetros do Controlador PID sintonizado por AG
const double ag_kp = -0.3;   // demo_kp= -1.042
const double ag_ki = -0;     // demo_ki= -0.721
const double ag_kd = -0.6;  // demo_kd= -0.586

//Coeficiente 1/Tt do Sistema AntiWindUp
double demo_inv_Tt = 1/sqrt(demo_td*demo_ti);

//Coeficientes Controlador Discreto I
//Numerador
double demo_idb0 = 0;
double demo_idb1 = demo_Ts_segundos;
//Denominador
double demo_ida0 = 1;
double demo_ida1 = -1;

//Coeficientes Controlador Discreto D
//Numerador
double demo_ddb0 = demo_n;
double demo_ddb1 = -demo_n;
//Denominador
double demo_dda0 = (demo_td+(demo_Ts_segundos*demo_n));
double demo_dda1 = -demo_td;

//Variável de Entrada do Controlador Proporcional
double demo_px = 0;

//Variável de Saída do Controlador Proporcional
double demo_py = 0;

//Variáveis de Entrada do Controlador Integral
double demo_ix   = 0;
double demo_ix_1 = 0;

//Variáveis de Saída do Controlador Integral
double demo_iy   = 0;
double demo_iy_1 = 0;

//Variáveis de Entrada do Controlador Derivativo
double demo_dx   = 0;
double demo_dx_1 = 0;

//Variáveis de Saída do Controlador Derivativo
double demo_dy   = 0;
double demo_dy_1 = 0;

//Variável de Entrada do Controlador
double demo_x = 0;

//Variável de Saída do Controlador
double demo_u = 0;

//Variável de Saída não Saturada do Controlador
double demo_uNs = 0;

//Variável de Saída da Planta
int demo_y = 0;

//Variável auxiliar de contagem;
volatile long int demo_tempo_1 = 0;

/* FIM - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO DEMO */

//Protótipo do Procedimento do Módulo Demo
void demo_modulo();

//Protótipo do Procedimento de Implementação do Controlador do Módulo Demo
void demo_computar();

/* INÍCIO - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO PID CONVENCIONAL */
//Tempo de Amostragem em ms
const int conv_Ts = 10;
//Tempo de Amostragem em ms
const double conv_Ts_segundos = 0.010;

//Parâmetros do Controlador Paralelo
double conv_kp;
double conv_ki;
double conv_kd;
double conv_ti;
double conv_td;

//Constante N do Filtro do Termo Derivativo
double conv_n = 2;

//Constante de Tempo de Reset do Termo Integral
double conv_invTt;

//Coeficientes do Controlador Integral
//Numerador - Potências de Z Crescente
double conv_idb0;
double conv_idb1;
//Denominador - Potências de Z Crescente
double conv_ida0;
double conv_ida1;

//Coeficientes do Controlador Derivativo com Filtro
//Numerador - Potências de Z Crescente
double conv_ddb0;
double conv_ddb1;
//Denominador - Potências de Z Crescente
double conv_dda0;
double conv_dda1;

//Variável de Entrada do Controlador Proporcional
double conv_px = 0;

//Variável de Saída do Controlador Proporcional
double conv_py = 0;

//Variáveis de Entrada do Controlador Integral
double conv_ix   = 0;
double conv_ix_1 = 0;

//Variáveis de Saída do Controlador Integral
double conv_iy   = 0;
double conv_iy_1 = 0;

//Variáveis de Entrada do Controlador Derivativo
double conv_dx   = 0;
double conv_dx_1 = 0;

//Variáveis de Saída do Controlador Derivativo
double conv_dy   = 0;
double conv_dy_1 = 0;

//Variável de Saída do Controlador
double conv_u = 0;

//Variável de Saída não Saturada do Controlador
double conv_uNs = 0;

//Variável de Saída da Planta
int conv_y = 0;

//Fonte do Sinal entrada do Bloco Proporcional E - erro, M - Sinal Medido na Saída da Planta
char  conv_fonteSinalProporcional = 'E';

//Variável Auxiliar para Implementação do tempo de Amostragem 
long volatile conv_tempo_anterior = 0;

//Protótipo do Procedimento do Módulo PID Convencional
void conv_modulo();

//Protótipo do Procedimento de Implementação do Controlador do Módulo PID Convencional
void conv_computar();

//Protótipo do Procedimento de Definição dos Coeficientes do Controlador do Módulo PID Convencional 
void conv_set_parametros();

/* FIM - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO PID CONVENCIONAL */

/* INÍCIO - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO DE MODELAGEM E PROJETO DE CONTROLADOR */
//Tempo de Amostragem em ms
const int proj_Ts = 100;
//Tempo de Amostragem em ms
const double proj_Ts_segundos = 0.100;

//Parâmetros da Planta
//Massa da Esfera [kg]
const double proj_mb = 0.103;
//Raio da Esfera [m]
const double proj_rb = 0.01465;
//Aceleração da Gravidade [m/s^2]
const double proj_g = 9.80511092;    
//Distância Entre o Centro do Eixo e o Ponto de Engate da Haste [m]
const double proj_d = 0.034; 
//Distância Entre o Ponto de Apoio da Barra e a Conexão com a Haste [m]
const double proj_l = 0.15425;
//Momento de Inércia Polar [kg*m^2]
const double proj_jb = 0.4*proj_mb*pow(proj_rb,2);
//Constante da Planta
const double proj_km = -((proj_mb*proj_g*proj_d*pow(proj_rb,2))/(proj_l*(proj_mb*pow(proj_rb,2)+proj_jb)));
//Ajuste para entrada em us do PWM e saída da planta de 0 a 1023
const double proj_kPlanta = proj_km*(0.09)*(PI/180)*2048;
//Atraso de Transporte [s]
const double proj_teta = 0.100;

//Parâmetros do Controlador
double proj_lambdaC;
double proj_lambdaF;
double proj_tf;
double proj_ti;
double proj_td;
double proj_kc;

//Constantes de Proporcionalidade Recebidas para Defininição de lambdaC e lambdaF
double proj_const_lambdaC = 2.5;
double proj_const_lambdaF = 1.0;

//Parâmetros do Filtro de SetPoint 1 Contínuo
//Numerador Potências de S Decrescente
double proj_f1cb0;
double proj_f1cb1;
double proj_f1cb2;
//Numerador Potências de S Decrescente
double proj_f1ca0;
double proj_f1ca1;
double proj_f1ca2;

//Coeficientes do Filtro de SetPoint 1 Discreto
//Numerador - Potências de Z Crescente
double proj_f1db0;
double proj_f1db1;
double proj_f1db2;
//Denominador - Potências de Z Crescente
double proj_f1da0;
double proj_f1da1;
double proj_f1da2;

//Parâmetros do Filtro de SetPoint 2 Contínuo
//Numerador Potências de S Decrescente
double proj_f2cb0;
double proj_f2cb1;
double proj_f2cb2;
//Denominador Potências de S Decrescente
double proj_f2ca0;
double proj_f2ca1;
double proj_f2ca2;

//Coeficientes do Filtro de SetPoint 2
//Numerador - Potências de Z Crescente
double proj_f2db0;
double proj_f2db1;
double proj_f2db2;
//Denominador - Potências de Z Crescente
double proj_f2da0;
double proj_f2da1;
double proj_f2da2;

//Coeficientes do Controlador Integral
//Numerador - Potências de Z Crescente
double proj_idb0;
double proj_idb1;
//Denominador - Potências de Z Crescente
double proj_ida0;
double proj_ida1;

//Coeficientes do Controlador Derivativo com Filtro
//Numerador - Potências de Z Crescente
double proj_ddb0;
double proj_ddb1;
//Denominador - Potências de Z Crescente
double proj_dda0;
double proj_dda1;

//Ganhos dos Controladores
double proj_kp;
double proj_ki;
double proj_kd;

//Variáveis de Entrada do Filtro de SetPoint 1
double proj_f1x   = 0;
double proj_f1x_1 = 0;
double proj_f1x_2 = 0;

//Variáveis de Saída do Filtro de Setpoint 1
double proj_f1y   = 0;
double proj_f1y_1 = 0;
double proj_f1y_2 = 0;

//Variáveis de Entrada do Filtro de SetPoint 2
double proj_f2x   = 0;
double proj_f2x_1 = 0;
double proj_f2x_2 = 0;

//Variáveis de Saída do Filtro de Setpoint 2
double proj_f2y   = 0;
double proj_f2y_1 = 0;
double proj_f2y_2 = 0;

//Variável de Entrada do Controlador Proporcional
double proj_px = 0;

//Variável de Saída do Controlador Proporcional
double proj_py = 0;

//Variáveis de Entrada do Controlador Integral
double proj_ix   = 0;
double proj_ix_1 = 0;

//Variáveis de Saída do Controlador Integral
double proj_iy   = 0;
double proj_iy_1 = 0;

//Variáveis de Entrada do Controlador Derivativo
double proj_dx   = 0;
double proj_dx_1 = 0;

//Variáveis de Saída do Controlador Derivativo
double proj_dy   = 0;
double proj_dy_1 = 0;

//Variável de Entrada do Controlador
double proj_x = 0;

//Variável de Saída do Controlador
double proj_u = 0;

//Variável de Saída não Saturada do Controlador
double proj_uNs = 0;

//Variável de Saída da Planta
double proj_y = 0;

//Variável Auxiliar para Implementação do tempo de Amostragem 
long volatile proj_tempo_anterior = 0;

//Inverso da Variável de Reset do Termo Integral
double proj_invTt = 0;


//Protótipo do Procedimento do Módulo de Modelagem e Projeto de Controlador
void proj_modulo();

//Protótipo do Procedimento de Implementação do Controlador do Módulo de Modelagem e Projeto de Controlador
void proj_computar();

//Protótipo do Procedimento de Definição dos Coeficientes do Controlador do Módulo de Modelagem e Projeto de Controlador
void proj_set_parametros();

/* FIM - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO DE MODELAGEM E PROJETO DE CONTROLADOR */

/* INÍCIO - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO DE MODELAGEM E PROJETO DA REDE NEURAL ARTIFICIAL */

const int PatternCount = 9;
const int InputNodes = 3;
const int HiddenNodes = 6; 
const int OutputNodes = 1;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5; 
const float Success = 0.00002;

float Input[PatternCount][InputNodes] = {
  { 1, 0, 0.5},  // bola a esquerda do setpoint
  { 1, 0, 0.25},
  { 1, 0, 0.75},
  { 0, 1, 0.5}, 	// bola a direita do set point
  { 0, 1, 0.25},
  { 0, 1, 0.75},
  { 0, 0, 0.5},  
  { 0, 0, 0.25},  
  { 0, 0, 0.75},  
}; 

const float Target[PatternCount][OutputNodes] = {
  { 0.25 },  
  { 0.125 },  
  { 0.375 },
  { 0.75 }, 
  { 0.625 },
  { 0.875 },
  { 0.5 },  
  { 0.5 },  
  { 0.5 },  
};

int i, j, m, n, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long  TrainingCycle;
float Rando;
float Error=1;
float Accum;
float nnOutMax;
float nnOutMin;


float Hidden[HiddenNodes];
float Output[OutputNodes];
float HiddenWeights[InputNodes+1][HiddenNodes];
float OutputWeights[HiddenNodes+1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes+1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes+1][OutputNodes];

void neuralnetwork_module();
void train_nn();
void toTerminal();
void InputToOutput(float, float, float);

/* FIM - VARIÁVEIS, CONSTANTES E PROTÓTIPOS DO MÓDULO DE MODELAGEM E PROJETO DA REDE NEURAL ARTIFICIAL */

/* INÍCIO - PROCEDIMENTO SETUP */
void setup() {
  //Configura a Velocidade da Comunicação Serial
  Serial.begin(115200);

  //Especifica a Saída Asssociado ao Servomotor
  bb_servo.attach(9);

  //Atribui Posição Neutra ao Servomotor
  bb_servo.write(1550);

  //Normaliza Parâmetros do Controlador I do Módulo Demo
  demo_idb0 = demo_idb0/demo_ida0;
  demo_idb1 = demo_idb1/demo_ida0;
  demo_ida1 = demo_ida1/demo_ida0;
  demo_ida0 = 1;

  //Normaliza Parâmetros do Controlador D do Módulo Demo
  demo_ddb0 = demo_ddb0/demo_dda0;
  demo_ddb1 = demo_ddb1/demo_dda0;
  demo_dda1 = demo_dda1/demo_dda0;
  demo_dda0 = 1;

  proj_set_parametros();

  ReportEvery1000 = 1;
}
/* FIM - PROCEDIMENTO SETUP */

/* INÍCIO - PROCEDIMENTO LOOP */
void loop() {
  //Variável de Seleção do Módulo
  int modulo_selecionado = -1;

  //Aguarda Recebimento do Módulo Selecionado
  while (Serial.available()==0){};

  //Faz a Leitura do Módulo
  modulo_selecionado = Serial.read();
	
  //Se Selecionado o Módulo Fuzzy - 'T'
  if (modulo_selecionado == 84) {
    //'T' - Conexão Estabelecida Corretamente com Módulo Controlador Fuzzy
    Serial.println("T");
    
    //Executar Módulo Controlador Fuzzy
    fuzzy_modulo();
  }

//Se Selecionado o Módulo Rede Neural - 'U'
  if (modulo_selecionado == 85) {
    //'T' - Conexão Estabelecida Corretamente com Módulo Controlador Rede Neural
    Serial.println("U");
    
    //Executar Módulo Controlador Rede Neural
    neuralnetwork_module();
  }

  //Se Selecionado o Módulo Manual - 'W'
  else if (modulo_selecionado == 87) {
    //'W' - Conexão Estabelecida Corretamente com Módulo Manual
    Serial.println("W");
    
    //Executar Módulo Manual
    manual_modulo();
  }
  //Se Selecionado o Módulo Controlador Padrão - 'X'
  else if (modulo_selecionado == 88) {
    //'X' - Conexão Estabelecida Corretamente com Módulo Controlador Padrão
    Serial.println("X");
    
    //Executar Módulo Controlador Padrão
    conv_modulo();
  }
  //Se Selecionado o Módulo de Modelagem - 'Y'
  else if (modulo_selecionado == 89) {
    //'Y' - Conexão Estabelecida Corretamente com Módulo Modelagem
    Serial.println("Y");
    
    //Executar Módulo de Modelagem
    proj_modulo();
  }
  //Se Selecionado o Módulo de Demonstração - 'Z'
  else if (modulo_selecionado == 90) {
    //'Z' - Conexão Estabelecida Corretamente com Módulo Demonstração
    Serial.println("Z");
    
    //Executar Módulo de Demonstração
    demo_modulo();
  }
}
/* FIM - PROCEDIMENTO LOOP */

/* INÍCIO - PROCEDIMENTO MÓDULO FUZZY */
void fuzzy_modulo() {
  int   leitura_serial = -1;
  int   leitura_pot = 0;
  int   soft_pot = 0;
  int   sinal_servo = 0;
  bool  comando = false;
  char  str_set_point[4];
  int   dist_set_point;
  int   perto=25;
  int   medio=50;
  int   longe=75;
  int   mlonge=100;

  while(true) {
    if (Serial.available() != 0) {
      leitura_serial = Serial.read();
      //'I' - Comando Iniciar
      if (leitura_serial == 73) {
        comando = true;
        soft_pot = analogRead(5);
        set_point = 250;
        y_V[0] = demo_y;
        y_V[1] = demo_y;
        y_V[2] = demo_y;
        y_V[3] = demo_y;
      }
      //'P' - Comando Parar
      else if (leitura_serial == 80) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        comando = false;
      }
      //'$' - Comando Sair
      else if(leitura_serial == 36) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        break;
      }
      //'L' - Comando Leituras
      else if(leitura_serial == 76) {
        //Caracter Indicativo de Leituras 
        Serial.print('L');
        Serial.print(',');
        //Sinal Lido do Sensor SoftPot
        Serial.print((int)soft_pot*2);
        Serial.print(',');
        //Sinal Lido do Potenciômetro
        Serial.print((int)leitura_pot);
        Serial.print(',');
        //Sinal Aplicado Ao Servo
        Serial.println((int)map(sinal_servo,-300,300,1050,2050));     
      }
	  //'S' - Atribuição do SetPoint
      else if (leitura_serial == 83) {
			//Aguarda Recebimento dos Bytes do valor de SetPoint
			while (Serial.available()<3){};
        
			str_set_point[0] = Serial.read();
			str_set_point[1] = Serial.read();
			str_set_point[2] = Serial.read();
			str_set_point[3] = '\n';

			double set_point_temp = set_point;
			set_point = atof(str_set_point);
			
			if (set_point > 500 || set_point < 0)
			  set_point = set_point_temp;

			Serial.flush();
		}
	  
      leitura_serial = -1;
    }

    if (comando == true) {
      
      //Atualização das Variáveis do Filtro do Sensor de Posição
      y_V[3] = y_V[2];
      y_V[2] = y_V[1];
      y_V[1] = y_V[0];
      //Leitura do Sinal do Sensor
      y_V[0] = analogRead(5);
      //Atualiza o Valor da Variável de Saída da Planta
      soft_pot = (y_V[0]+y_V[1]+y_V[2]+y_V[3])>>2;
      //Aplica Função Parábola para Linearizar o Sensor
      //soft_pot = (soft_pot-((((soft_pot-h)*(soft_pot-h))/(4*p))+k));
      soft_pot = (int)map(soft_pot,0,1000,0,500);

    dist_set_point = soft_pot -set_point;

	  if(dist_set_point>=120)                       //distancia maior que 120 mm
	    sinal_servo = mlonge; // inclina a barra em 20 graus
	  if(dist_set_point<120 && dist_set_point>=70) //distancia maior que 70 mm
		  sinal_servo = longe; // inclina a barra em 15 graus
    if(dist_set_point<70 && dist_set_point>=30) //distancia maior que 30 mm
		  sinal_servo = medio; // inclina a barra em 10 graus
    if(dist_set_point<30 && dist_set_point>=10) //distancia maior que 10 mm
		  sinal_servo = perto; // inclina a barra em 5 graus

    if(dist_set_point<10 && dist_set_point>-10 ) //distancia menor que 10 mm e maior que -10 mm
		  sinal_servo = 0; // inclina a barra em 0 graus
    
    if(dist_set_point<=-120)                        //distancia maior que -120 mm
	    sinal_servo = -mlonge; // inclina a barra em -20 graus 
	  if(dist_set_point>-120 && dist_set_point<=-70) //distancia maior que -70 mm
		  sinal_servo = -longe; // inclina a barra em -15 graus
    if(dist_set_point>-70 && dist_set_point<=-30) //distancia maior que -30 mm
		  sinal_servo = -medio; // inclina a barra em -10 graus
    if(dist_set_point>-30 && dist_set_point<=-10) //distancia maior que -10 mm
		  sinal_servo = -perto; // inclina a barra em -5 graus

    
  #ifdef DEBUG      //utilizado na amostragem do teste comparativo
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println((int)soft_pot);
  }
  #endif

    //Atribuição do Sinal de Controle ao ServoMotor
      bb_servo.write((int)sinal_servo+1550);
    
    }
    else {
      //Atribui Posição Neutra ao Servo
      bb_servo.write(1550);
    }
  }
}
/* FIM - PROCEDIMENTO MÓDULO FUZZY */

/* INÍCIO - PROCEDIMENTO MÓDULO REDE NEURAL */
void neuralnetwork_module() {
  int   leitura_serial = -1;
  int   leitura_pot = 0;
  int   soft_pot = 0;
  int   sinal_servo = 0; //int
  float aux_servo;
  bool  comando = false;
  
  char  str_set_point[4];
  int   dist_set_point;
  float TestInput[] = {0, 0};
	float LL1, LL2, LL3;

  while(true) {
    if (Serial.available() != 0) {
      leitura_serial = Serial.read();
      //'I' - Comando Iniciar
      if (leitura_serial == 73) {
        comando = true;
        soft_pot = analogRead(5);
        set_point = 250;
        y_V[0] = demo_y;
        y_V[1] = demo_y;
        y_V[2] = demo_y;
        y_V[3] = demo_y;
      }
      //'P' - Comando Parar
      else if (leitura_serial == 80) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        comando = false;
      }
      //'$' - Comando Sair
      else if(leitura_serial == 36) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        break;
      }
      //'L' - Comando Leituras
      else if(leitura_serial == 76) {
        //Caracter Indicativo de Leituras 
        Serial.print('L');
        Serial.print(',');
        //Sinal Lido do Sensor SoftPot
        Serial.print((int)soft_pot*2);
        Serial.print(',');
        //Sinal Lido do Potenciômetro
        Serial.print((int)leitura_pot);
        Serial.print(',');
        //Sinal Aplicado Ao Servo
        Serial.println((int)map(sinal_servo,-300,300,1050,2050));
      }
	    //'S' - Atribuição do SetPoint
      else if(leitura_serial == 83) {
			//Aguarda Recebimento dos Bytes do valor de SetPoint
			while (Serial.available()<3){};
        
			str_set_point[0] = Serial.read();
			str_set_point[1] = Serial.read();
			str_set_point[2] = Serial.read();
			str_set_point[3] = '\n';

			double set_point_temp = set_point;
			set_point = atof(str_set_point);
			
			if (set_point > 500 || set_point < 0)
			  set_point = set_point_temp;

			Serial.flush();
      }
    //'T' - Treino da rede neural 
      else if (leitura_serial == 84) {
        Serial.println("Treino da rede neural selecionado");
        train_nn();
        
        toTerminal();
        Serial.println("Treino da rede neural FINALIZADO");
      }  
    }
	  
      

    if (comando && Error <= Success) {
     
      //Atualização das Variáveis do Filtro do Sensor de Posição
      y_V[3] = y_V[2];
      y_V[2] = y_V[1];
      y_V[1] = y_V[0];
      //Leitura do Sinal do Sensor
      y_V[0] = analogRead(5);
      //Atualiza o Valor da Variável de Saída da Planta
      soft_pot = (y_V[0]+y_V[1]+y_V[2]+y_V[3])>>2;
      //Aplica Função Parábola para Linearizar o Sensor
      //soft_pot = (soft_pot-((((soft_pot-h)*(soft_pot-h))/(4*p))+k));
      soft_pot = (int)map(soft_pot,0,1000,0,500);               
      
      dist_set_point = soft_pot - set_point;

  #ifdef DEBUG    //utilizado na amostragem do teste comparativo
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.println((int)soft_pot);
      }
  #endif
      
      if(dist_set_point<=0){
        LL1 = -dist_set_point;
        LL2 = 0;
      }
      else{
        LL1 = 0;
        LL2 = dist_set_point;
      }
      
     // mapeamento das variaveis para adequação na RNA 
    LL1 = map((float)LL1, 0,(500-set_point), 0, 100);
    LL2 = map((float)LL2, 0, set_point, 0, 100);
    LL3 = map((float)LL3, 0, 500, 0, 100);   

    LL1 = constrain(LL1, 0, 100);
    LL2 = constrain(LL2, 0, 100);
    LL3 = constrain(LL3, 0, 100);
   
    TestInput[0] = float(LL1) / 100;
    TestInput[1] = float(LL2) / 100;
    TestInput[2] = float(LL3) / 100;

    // chamda da RNA inserindo os dados nos neuronios de entrada
    InputToOutput(TestInput[0], TestInput[1], TestInput[2]); //INPUT to ANN to obtain OUTPUT    */
    
    // mapeamento das variaveis para adequação na RNA 
    aux_servo = Output[0] * 100 *100 *100;
    sinal_servo = map(aux_servo, nnOutMin, nnOutMax, 100, -100);
        
    //Atribuição do Sinal de Controle ao ServoMotor
    bb_servo.write((int)sinal_servo+1550);
    
            
    }
    else if (comando && (Error > Success)) {
    
      //Rede Neural ainda não foi treinada, treinar antes de executar
      Serial.println("Rede Neural ainda não treinada");
      comando = false;
    }
    else {
      //Atribui Posição Neutra ao Servo
      bb_servo.write(1550);
    }

    leitura_serial = -1;
  }
}

/* FIM - PROCEDIMENTO MÓDULO REDE NEURAL */

/* INÍCIO - PROCEDIMENTO PROCESSAMENTO DA SAIDA DA NEURAL NETWORK */
void InputToOutput(float In1, float In2, float In3)
{
  float TestInput[] = {0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;

  /******************************************************************
    Compute hidden layer activations
  ******************************************************************/

  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    Accum = HiddenWeights[InputNodes][i] ;
    for ( j = 0 ; j < InputNodes ; j++ ) {
      Accum += TestInput[j] * HiddenWeights[j][i] ;
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/

  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Accum = OutputWeights[HiddenNodes][i] ;
    for ( j = 0 ; j < HiddenNodes ; j++ ) {
      Accum += Hidden[j] * OutputWeights[j][i] ;
    }
    Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
  }

}
/* FIM - PROCEDIMENTO PROCESSAMENTO DA SAIDA DA NEURAL NETWORK */

/* INÍCIO - PROCEDIMENTO TRAINS THE NEURAL NETWORK */
void train_nn() {

  /******************************************************************
    Initialize HiddenWeights and ChangeHiddenWeights
  ******************************************************************/
  
  //      Serial.println("Initialize HiddenWeights and ChangeHiddenWeights");
  
  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    for ( j = 0 ; j <= InputNodes ; j++ ) {
      ChangeHiddenWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  
  /******************************************************************
    Initialize OutputWeights and ChangeOutputWeights
  ******************************************************************/
  //      Serial.println("Initialize OutputWeights and ChangeOutputWeights");
  for ( i = 0 ; i < OutputNodes ; i ++ ) {
    for ( j = 0 ; j <= HiddenNodes ; j++ ) {
      ChangeOutputWeights[j][i] = 0.0 ;
      Rando = float(random(100)) / 100;
      OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }

  /******************************************************************
    Begin training
  ******************************************************************/
  for ( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {

    /******************************************************************
      Randomize order of training patterns
    ******************************************************************/

    for ( m = 0 ; m < PatternCount ; m++) {

      n = random(PatternCount);
      r = RandomizedIndex[m] ;
      RandomizedIndex[m] = RandomizedIndex[n] ;
      RandomizedIndex[n] = r ;
    }
    Error = 0.0 ;
    /******************************************************************
      Cycle through each training pattern in the randomized order
    ******************************************************************/
    for ( n = 0 ; n < PatternCount ; n++ ) {
      m = RandomizedIndex[n];

      /******************************************************************
        Compute hidden layer activations
      ******************************************************************/
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = HiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          Accum += Input[m][j] * HiddenWeights[j][i] ;
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
      }
      

      /******************************************************************
        Compute output layer activations and calculate errors
      ******************************************************************/
      for ( i = 0 ; i < OutputNodes ; i++ ) {

        Accum = OutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          Accum += Hidden[j] * OutputWeights[j][i] ;
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
        OutputDelta[i] = (Target[m][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;
        Error += 0.5 * (Target[m][i] - Output[i]) * (Target[m][i] - Output[i]) ;
      }

      
      /******************************************************************
        Backpropagate errors to hidden layer
      ******************************************************************/

      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = 0.0 ;
        for ( j = 0 ; j < OutputNodes ; j++ ) {
          Accum += OutputWeights[i][j] * OutputDelta[j] ;
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;
      }
      

      /******************************************************************
        Update Inner-->Hidden Weights
      ******************************************************************/
      for ( i = 0 ; i < HiddenNodes ; i++ ) {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;
        for ( j = 0 ; j < InputNodes ; j++ ) {
          ChangeHiddenWeights[j][i] = LearningRate * Input[m][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;
        }
      }
      
      /******************************************************************
        Update Hidden-->Output Weights
      ******************************************************************/
      for ( i = 0 ; i < OutputNodes ; i ++ ) {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;
        for ( j = 0 ; j < HiddenNodes ; j++ ) {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;
          OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
        }
      }
      
    }

    /******************************************************************
      Every cycles send data to terminal for display 
    ******************************************************************/

      Serial.print ("TrainingCycle: ");
      Serial.print (TrainingCycle);
      Serial.print ("  Error = ");
      Serial.println (Error, DEC);
      
    /******************************************************************
      If error rate is less than pre-determined threshold then end
    ******************************************************************/

    if ( Error < Success ) break ;
  }
}

//DISPLAYS INFORMATION WHILE TRAINING
void toTerminal()
{

  for ( m = 0 ; m < PatternCount ; m++ ) {
    Serial.println();
    Serial.print ("  Training Pattern: ");
    Serial.println (m);
    Serial.print ("    Input ");
    for ( i = 0 ; i < InputNodes ; i++ ) {
      Serial.print (Input[m][i], 2);
      Serial.print (" ");
    }
    Serial.print ("  Target ");
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Serial.print (Target[m][i], 2);
      Serial.print (" ");
    }
    /******************************************************************
      Compute hidden layer activations
    ******************************************************************/

    for ( i = 0 ; i < HiddenNodes ; i++ ) {
      Accum = HiddenWeights[InputNodes][i] ;
      for ( j = 0 ; j < InputNodes ; j++ ) {
        Accum += Input[m][j] * HiddenWeights[j][i] ;
        }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }

    /******************************************************************
      Compute output layer activations and calculate errors
    ******************************************************************/

    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Accum = OutputWeights[HiddenNodes][i] ;
      for ( j = 0 ; j < HiddenNodes ; j++ ) {
        Accum += Hidden[j] * OutputWeights[j][i] ;
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
    }
    Serial.print ("  Output ");
    for ( i = 0 ; i < OutputNodes ; i++ ) {
      Serial.print (Output[i], DEC);
      Serial.print (" ");
      if(m==4)
          nnOutMin = Output[i]*100*100*100;
      else if(m==2)
          nnOutMax = Output[i]*100*100*100;
     }

  }
  Serial.println(" ");
  
}
/* FIM - PROCEDIMENTO TRAINS THE NEURAL NETWORKL */


/* INÍCIO - PROCEDIMENTO MÓDULO MANUAL */
void manual_modulo() {
  int   leitura_serial = -1;
  int   leitura_pot = 0;
  int   soft_pot = 0;
  int   sinal_servo = 0;
  bool  comando = false;


  while(true) {
    if (Serial.available() != 0) {
      leitura_serial = Serial.read();
      //'I' - Comando Iniciar
      if (leitura_serial == 73) {
        comando = true;
        soft_pot = analogRead(5);
        y_V[0] = demo_y;
        y_V[1] = demo_y;
        y_V[2] = demo_y;
        y_V[3] = demo_y;
      }
      //'P' - Comando Parar
      else if (leitura_serial == 80) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        comando = false;
      }
      //'$' - Comando Sair
      else if(leitura_serial == 36) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        break;
      }
      //'L' - Comando Leituras
      else if(leitura_serial == 76) {
        //Caracter Indicativo de Leituras 
        Serial.print('L');
        Serial.print(',');
        //Sinal Lido do Sensor SoftPot
        Serial.print((int)soft_pot);
        Serial.print(',');
        //Sinal Lido do Potenciômetro
        Serial.print((int)leitura_pot);
        Serial.print(',');
        //Sinal Aplicado Ao Servo
        Serial.println((int)map(sinal_servo,-300,300,1050,2050));
      }
      leitura_serial = -1;
    }

    if (comando == true) {
      
      //Atualização das Variáveis do Filtro do Sensor de Posição
      y_V[3] = y_V[2];
      y_V[2] = y_V[1];
      y_V[1] = y_V[0];
      //Leitura do Sinal do Sensor
      y_V[0] = analogRead(5);
      //Atualiza o Valor da Variável de Saída da Planta
      soft_pot = (y_V[0]+y_V[1]+y_V[2]+y_V[3])>>2;
      //Aplica Função Parábola para Linearizar o Sensor
      //soft_pot = (soft_pot-((((soft_pot-h)*(soft_pot-h))/(4*p))+k));

      //Atualização das Variáveis do Filtro do Potenciômetro
      y_P[3] = y_P[2];
      y_P[2] = y_P[1];
      y_P[1] = y_P[0];
      //leitura do Sinal do Potenciômetro
      y_P[0] = analogRead(0);
      //Atualiza o Valor do Sinal do Potenciômetro
      leitura_pot = (y_P[0]+y_P[1]+y_P[2]+y_P[3])>>2;

      //Atualização do Sinal Aplicado ao Servo
      sinal_servo = map(leitura_pot,0,1023,300,-300);

      //Atribuição do Sinal de Controle ao ServoMotor
      bb_servo.write((int)sinal_servo+1550);

    }
    else {
      //Atribui Posição Neutra ao Servo
      bb_servo.write(1550);
    }
  }
}
/* FIM - PROCEDIMENTO MÓDULO MANUAL */

/* INICIO - PROCEDIMENTO MÓDULO DEMO */
void demo_modulo() {
  int   leitura_serial = -1;
  int   leitura_pot = 0;
  bool  comando = false;
  char  str_set_point[4];
  
  while(true) {
    if (Serial.available() != 0) {
      leitura_serial = Serial.read();
      //'I' - Comando Iniciar
      if (leitura_serial == 73) {
        comando = true;
        demo_y = analogRead(5);
        y_V[0] = demo_y;
        y_V[1] = demo_y;
        y_V[2] = demo_y;
        y_V[3] = demo_y;
      }
      //'P' - Comando Parar
      else if (leitura_serial == 80) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        comando = false;
      }
      //'$' - Comando Sair
      else if(leitura_serial == 36) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        break;
      }
      //'L' - Comando Leituras
      else if(leitura_serial == 76) {
        //Caracter Indicativo de Leituras 
        Serial.print('L');
        Serial.print(',');
        //Sinal Lido do Sensor SoftPot
        Serial.print((int)demo_y);
        Serial.print(',');
        //Sinal Lido do Potenciômetro
        Serial.print((int)leitura_pot);
        Serial.print(',');
        //Sinal Aplicado Ao Servo
        Serial.println((int)map(demo_u,-300,300,1050,2050));
      }
      //'S' - Atribuição do SetPoint
      else if (leitura_serial == 83) {
        //Aguarda Recebimento dos Bytes do valor de SetPoint
        while (Serial.available()<3){};
        
        str_set_point[0] = Serial.read();
        str_set_point[1] = Serial.read();
        str_set_point[2] = Serial.read();
        str_set_point[3] = '\n';

        double set_point_temp = set_point;
        set_point = atof(str_set_point);
        set_point = map(set_point,0,500,0,1023);

        if (set_point > 1023 || set_point < 0)
          set_point = set_point_temp;

        Serial.flush();
      }
      leitura_serial = -1;
    }

    if (comando == true) {
      //Verifica Tempo de Amostragem
      if(millis() - demo_tempo_1 >= demo_Ts){
        //Executa Controlador PID
        demo_computar();
              
        
        //Atribuição do Sinal de Controle ao ServoMotor
        bb_servo.write((int)demo_u+1550);

        //Atualiza tempo
        demo_tempo_1 = millis();
      }

      //Atualização das Variáveis do Filtro do Sensor de Posição
      y_V[3] = y_V[2];
      y_V[2] = y_V[1];
      y_V[1] = y_V[0];
      //Leitura do Sinal do Sensor
      y_V[0] = analogRead(5);
      
    

      //Atualiza o Valor da Variável de Saída da Planta
      demo_y = (y_V[0]+y_V[1]+y_V[2]+y_V[3])>>2;
      //Aplica Função Parábola para Linearizar o Sensor
      //demo_y = (demo_y-((((demo_y-h)*(demo_y-h))/(4*p))+k));
      
      
  #ifdef DEBUG     //utilizado na amostragem do teste comparativo
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.println((int)map(demo_y,0,1000,0,500));
      }
  #endif

      //------------------------------------------------------------------------------------------
         
      
      //Sinal Lido do Sensor SoftPot
         
      //------------------------------------------------------------------------------------------
    }
    else {
      //Atribui Posição Neutra ao Servo
      bb_servo.write(1550);
    }
  }
}
/* FIM - PROCEDIMENTO MÓDULO DEMO */

/* INÍCIO - PROCEDIMENTO COMPUTAR DO MÓDULO DEMO */
void demo_computar() {  
  
  /* Início - Controlador Proporcional*/
  demo_px = -demo_y;
  demo_py = demo_kp*demo_px;
  /* Fim - Controlador Proporcional */

  /*Início - Controlador Integral*/
  demo_ix = set_point-demo_y;
  demo_ix = demo_ix*demo_ki;
  //demo_ix = demo_ix + (demo_inv_Tt * (demo_u-demo_uNs));
  demo_iy = demo_idb0*demo_ix + demo_idb1*demo_ix_1 - demo_ida1*demo_iy_1 + (demo_inv_Tt * (demo_u-demo_uNs));
  //demo_iy = demo_iy/demo_ida0;
  demo_ix_1 = demo_ix;
  demo_iy_1 = demo_iy;
  /*Fim - Controlador Integral*/

  /*Início - Controlador Derivativo*/
  demo_dx = -demo_y;
  demo_dx = demo_kd*demo_dx;
  demo_dy = demo_ddb0*demo_dx + demo_ddb1*demo_dx_1 - demo_dda1*demo_dy_1;
  demo_dx_1 = demo_dx;
  demo_dy_1 = demo_dy;
  //demo_dy = demo_dy/demo_dda0;
  /*Fim - Controlador Derivativo*/

  //Saída do Controlador - Sinal de Controle
  demo_u = demo_py+demo_iy+demo_dy;
  //Saída do controlador Não Saturada
  demo_uNs = demo_u;

  //Saturação do Sinal de Controle (-30 a 30 graus)
  if(demo_u >= 300)
    demo_u = 300;
  else if(demo_u <= -300)
    demo_u = -300;
}
/* FIM - PROCEDIMENTO COMPUTAR DO MÓDULO DEMO */

/* INICIO - PROCEDIMENTO MÓDULO PID CONVENCIONAL */
void conv_modulo() {
  int   leitura_serial = -1;
  int   leitura_pot = 0;
  bool  comando = false;
  char  str_set_point[4];
  char  str_P[8];
  char  str_I[9];
  char  str_D[10];
  char  str_N[3];

  while(true) {
    if (Serial.available() != 0) {
      leitura_serial = Serial.read();
      //'I' - Comando Iniciar
      if (leitura_serial == 73) {
        comando = true;
        conv_y = analogRead(5);
        y_V[0] = conv_y;
        y_V[1] = conv_y;
        y_V[2] = conv_y;
        y_V[3] = conv_y;
      }
      //'P' - Comando Parar
      else if (leitura_serial == 80) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        comando = false;
      }
      //'$' - Comando Sair
      else if(leitura_serial == 36) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        break;
      }
      //'L' - Comando Leituras
      else if(leitura_serial == 76) {
        //Caracter Indicativo de Leituras 
        Serial.print('L');
        Serial.print(',');
        //Sinal Lido do Sensor SoftPot
        Serial.print((int)conv_y);
        Serial.print(',');
        //Sinal Lido do Potenciômetro
        Serial.print((int)leitura_pot);
        Serial.print(',');
        //Sinal Aplicado Ao Servo
        Serial.println((int)map(conv_u,-300,300,1050,2050));
      }
      //'S' - Atribuição do SetPoint
      else if (leitura_serial == 83) {
        //Aguarda Recebimento dos Bytes do valor de SetPoint
        while (Serial.available()<3){};
        
        str_set_point[0] = Serial.read();
        str_set_point[1] = Serial.read();
        str_set_point[2] = Serial.read();
        str_set_point[3] = '/n';

        Serial.println(str_set_point);
        
        double set_point_temp = set_point;
        set_point = atof(str_set_point);
        set_point = map(set_point,0,500,0,1023);

        if (set_point > 1023 || set_point < 0)
          set_point = set_point_temp;
        
        Serial.println(set_point);
        Serial.flush();
      }
      //'G' - Atribuição dos Ganhos Kp - Ki - Kd - N - Fonte de Sinal do Termo Proporcional
      else if (leitura_serial == 71) {
        //Aguarda Recebimento dos Bytes dos Valores do Ganhos
        while (Serial.available()<24){};
        
        str_P[0] = Serial.read();
        str_P[1] = Serial.read();
        str_P[2] = Serial.read();
        str_P[3] = Serial.read();
        str_P[4] = Serial.read();
        str_P[5] = Serial.read();
        str_P[6] = Serial.read();
        str_P[7] = '\n';
        
        str_I[0] = Serial.read();
        str_I[1] = Serial.read();
        str_I[2] = Serial.read();
        str_I[3] = Serial.read();
        str_I[4] = Serial.read();
        str_I[5] = Serial.read();
        str_I[6] = Serial.read();
        str_I[7] = '\n';
        
        str_D[0] = Serial.read();
        str_D[1] = Serial.read();
        str_D[2] = Serial.read();
        str_D[3] = Serial.read();
        str_D[4] = Serial.read();
        str_D[5] = Serial.read();
        str_D[6] = Serial.read();
        str_D[7] = '\n';

        str_N[0] = Serial.read();
        str_N[1] = Serial.read();
        str_N[2] = '\n';
        
        conv_fonteSinalProporcional = Serial.read();
        
        conv_kp = atof(str_P) > 000.000 && atof(str_P) <= 100.000 ? -1*atof(str_P) : conv_kp;
        conv_ki = atof(str_I) > 000.000 && atof(str_I) <= 100.000 ? -1*atof(str_I) : conv_ki;
        conv_kd = atof(str_D) > 000.000 && atof(str_D) <= 100.000 ? -1*atof(str_D) : conv_kd;

        conv_n = atof(str_N) > 0.0 && atof(str_N) <= 100.0 ? atof(str_N) : conv_n;

        Serial.println(conv_kp);
        Serial.println(conv_ki);
        Serial.println(conv_kd);
        Serial.println(conv_n);
        Serial.println(conv_fonteSinalProporcional);

        if (conv_fonteSinalProporcional != 'E' && conv_fonteSinalProporcional != 'M')
          conv_fonteSinalProporcional = 'E';

        conv_set_parametros();
      }
      leitura_serial = -1;
    }

    if (comando == true) {
      //Verifica Tempo de Amostragem
      if(millis() - conv_tempo_anterior >= conv_Ts) {
        //Executa Controlador PID
        conv_computar();
        
        //Atribuição do Sinal de Controle ao ServoMotor
        bb_servo.write((int)conv_u+1550);

        //Atualiza tempo
        conv_tempo_anterior = millis();
        
      }

      //Atualização das Variáveis do Filtro do Sensor de Posição
      y_V[3] = y_V[2];
      y_V[2] = y_V[1];
      y_V[1] = y_V[0];
      //Leitura do Sinal do Sensor
      y_V[0] = analogRead(5);
      //Atualiza o Valor da Variável de Saída da Planta
      conv_y = (y_V[0]+y_V[1]+y_V[2]+y_V[3])>>2;
      //Aplica Função Parábola para Linearizar o Sensor
      //conv_y = (conv_y-((((conv_y-h)*(conv_y-h))/(4*p))+k));

      
    }
    else {
      //Atribui Posição Neutra ao Servo
      bb_servo.write(1550);
    }
  }
}
/* FIM - PROCEDIMENTO MÓDULO PID CONVENCIONAL */

/* INÍCIO - PROCEDIMENTO SET COEFICIENTES DOS PARÂMETROS DO MÓDULO PID CONVENCIONAL */
void conv_set_parametros() {
  //Constante de Tempo de Integração
  conv_ti = conv_kp/conv_ki;

  //Constante de Tempo Derivativo
  conv_td = conv_kd/conv_kp;

  //Constante de Tempo de Reset do Termo Integral
  conv_invTt = 1.0/sqrt(conv_td*conv_ti);

  //Coeficientes do Controlador Integral
  //Numerador - Potências de Z Crescente
  conv_idb0 = 0.0;
  conv_idb1 = conv_Ts_segundos;
  //Denominador - Potências de Z Crescente
  conv_ida0 = 1.0;
  conv_ida1 = -1.0;

  //Normaliza Parâmetros do Termo Integral
  conv_idb0 = conv_idb0/conv_ida0;
  conv_idb1 = conv_idb1/conv_ida0;
  conv_ida1 = conv_ida1/conv_ida0;
  conv_ida0 = 1.0;

  //Coeficientes do Controlador Derivativo com Filtro
  //Numerador - Potências de Z Crescente
  conv_ddb0 = conv_n;
  conv_ddb1 = -conv_n;
  //Denominador - Potências de Z Crescente
  conv_dda0 = (conv_td+(conv_Ts_segundos*conv_n));
  conv_dda1 = -conv_td;

  //Normaliza Parâmetros do Termo Derivativo
  conv_ddb0 = conv_ddb0/conv_dda0;
  conv_ddb1 = conv_ddb1/conv_dda0;
  conv_dda1 = conv_dda1/conv_dda0;
  conv_dda0 = 1;
/*
  Serial.println(conv_ti);
  Serial.println(conv_td);
  Serial.println(conv_invTt);
  
  Serial.println(conv_idb0);
  Serial.println(conv_idb1);
  Serial.println(conv_ida0);
  Serial.println(conv_ida1);

  Serial.println(conv_ddb0);
  Serial.println(conv_ddb1);
  Serial.println(conv_dda0);
  Serial.println(conv_dda1);
*/
}
/* FIM - PROCEDIMENTO SET COEFICIENTES DOS PARÂMETROS DO MÓDULO PID CONVENCIONAL */

/* INÍCIO - PROCEDIMENTO COMPUTAR COEFICIENTES DO CONTROLADOR PID CONVENCIONAL */
void conv_computar() {
  /* Início - Controlador Proporcional*/
  if (conv_fonteSinalProporcional ==  'E')
    conv_px = set_point - conv_y;
  else if(conv_fonteSinalProporcional == 'M')
    conv_px = -conv_y;
  else
    conv_px = set_point - conv_y;
    
  conv_py = conv_kp*conv_px;
  /* Fim - Controlador Proporcional */

  /*Início - Controlador Integral*/
  conv_ix = set_point-conv_y;
  conv_ix = conv_ix*conv_ki;
  conv_ix = conv_ix + (conv_invTt * (conv_u-conv_uNs));
  conv_iy = conv_idb0*conv_ix + conv_idb1*conv_ix_1 - conv_ida1*conv_iy_1;
  //conv_iy = conv_iy/conv_ida0;
  conv_ix_1 = conv_ix;
  conv_iy_1 = conv_iy;
  /*Fim - Controlador Integral*/

  /*Início - Controlador Derivativo*/
  conv_dx = -conv_y;
  conv_dx = conv_kd*conv_dx;
  conv_dy = conv_ddb0*conv_dx + conv_ddb1*conv_dx_1 - conv_dda1*conv_dy_1;
  conv_dx_1 = conv_dx;
  conv_dy_1 = conv_dy;
  //conv_dy = conv_dy/conv_dda0;
  /*Fim - Controlador Derivativo*/

  //Saída do Controlador - Sinal de Controle
  conv_u = conv_py+conv_iy+conv_dy;
  //Saída do controlador Não Saturada
  conv_uNs = conv_u;

  //Saturação do Sinal de Controle (-30 a 30 graus)
  if(conv_u >= 300)
    conv_u = 300;
  else if(conv_u <= -300)
    conv_u = -300;
} 
/* FIM - PROCEDIMENTO COMPUTAR COEFICIENTES DO CONTROLADOR PID CONVENCIONAL */

/* INÍCIO - PROCEDIMENTO MÓDULO DE MODELAGEM E PROJETO DE CONTROLADOR */
void proj_modulo() {
  int   leitura_serial = -1;
  int   leitura_pot = 0;
  bool  comando = false;
  char  str_set_point[4];
  char  str_lambdaC[7];
  char  str_lambdaF[7];

  while(true) {
    if (Serial.available() != 0) {
      leitura_serial = Serial.read();
      //'I' - Comando Iniciar
      if (leitura_serial == 73) {
        comando = true;
        proj_y = analogRead(5);
        y_V[0] = proj_y;
        y_V[1] = proj_y;
        y_V[2] = proj_y;
        y_V[3] = proj_y;
      }
      //'P' - Comando Parar
      else if (leitura_serial == 80) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        comando = false;
      }
      //'$' - Comando Sair
      else if(leitura_serial == 36) {
        //Atribuição Posição Neutra do Servo
        bb_servo.write(1550);
        break;
      }
      //'L' - Comando Leituras
      else if(leitura_serial == 76) {
        //Caracter Indicativo de Leituras 
        Serial.print('L');
        Serial.print(',');
        //Sinal Lido do Sensor SoftPot
        Serial.print((int)proj_y);
        Serial.print(',');
        //Sinal Lido do Potenciômetro
        Serial.print((int)leitura_pot);
        Serial.print(',');
        //Sinal Aplicado Ao Servo
        Serial.println((int)map(proj_u,-300,300,1050,2050));
      }
      //'S' - Atribuição do SetPoint
      else if (leitura_serial == 83) {
        //Aguarda Recebimento dos Bytes do valor de SetPoint
        while (Serial.available()<3){};
        
        str_set_point[0] = Serial.read();
        str_set_point[1] = Serial.read();
        str_set_point[2] = Serial.read();
        str_set_point[3] = '\n';

        //Serial.println(str_set_point);
        
        double set_point_temp = set_point;
        set_point = atof(str_set_point);
        set_point = map(set_point,0,500,0,1023);

        if (set_point > 1023 || set_point < 0)
          set_point = set_point_temp;
        
        //Serial.println(set_point);
        Serial.flush();
      }
      //'G' - Atribuição das Variáveis de Sintonia LambdaC e LambdaF
      else if (leitura_serial == 71) {
        //Aguarda Recebimento dos Bytes dos Valores do Ganhos
        while (Serial.available()<12){};
        
        str_lambdaC[0] = Serial.read();
        str_lambdaC[1] = Serial.read();
        str_lambdaC[2] = Serial.read();
        str_lambdaC[3] = Serial.read();
        str_lambdaC[4] = Serial.read();
        str_lambdaC[5] = Serial.read();
        str_lambdaC[6] = '\n';
        
        str_lambdaF[0] = Serial.read();
        str_lambdaF[1] = Serial.read();
        str_lambdaF[2] = Serial.read();
        str_lambdaF[3] = Serial.read();
        str_lambdaF[4] = Serial.read();
        str_lambdaF[5] = Serial.read();
        str_lambdaF[6] = '\n';
        
        proj_const_lambdaC = atof(str_lambdaC) > 000.00 && atof(str_lambdaC) <= 100.00 ? atof(str_lambdaC) : proj_const_lambdaC;
        proj_const_lambdaF= atof(str_lambdaF) > 000.00 && atof(str_lambdaF) <= 100.00 ? atof(str_lambdaF) : proj_const_lambdaF;

        //Serial.println(proj_const_lambdaC);
        //Serial.println(proj_const_lambdaF);

        if (proj_const_lambdaC < 0 || proj_const_lambdaC > 100)
          proj_const_lambdaC = 2.5;

        if (proj_const_lambdaF < 0 || proj_const_lambdaF > 100)
          proj_const_lambdaF = 1.0;

        proj_set_parametros();
      }
      leitura_serial = -1;
    }

    if (comando == true) {
      //Verifica Tempo de Amostragem
      if(millis() - proj_tempo_anterior >= proj_Ts) {
        //Executa Controlador PID
        proj_computar();
        
        //Atribuição do Sinal de Controle ao ServoMotor
        bb_servo.write((int)proj_u+1550);

        //Atualiza tempo
        proj_tempo_anterior = millis();
        
      }

      //Atualização das Variáveis do Filtro do Sensor de Posição
      y_V[3] = y_V[2];
      y_V[2] = y_V[1];
      y_V[1] = y_V[0];
      //Leitura do Sinal do Sensor
      y_V[0] = analogRead(5);
      //Atualiza o Valor da Variável de Saída da Planta
      proj_y = (y_V[0]+y_V[1]+y_V[2]+y_V[3])>>2;
      //Aplica Função Parábola para Linearizar o Sensor
      //proj_y = (proj_y-((((proj_y-h)*(proj_y-h))/(4*p))+k));

    }
    else {
      //Atribui Posição Neutra ao Servo
      bb_servo.write(1550);
    }
  }
}
/* FIM - PROCEDIMENTO MÓDULO DE MODELAGEM E PROJETO DE CONTROLADOR */

/* INÍCIO - PROCEDIMENTO COMPUTAR COEFICIENTES DO CONTROLADOR DE MODELAGEM E PROJETO DE CONTROLADOR */
void proj_computar() {
  /*Início - Filtro de setPoint 1*/
  proj_f1x = set_point;
  //Serial.print(proj_f1db0);
  //Serial.print(proj_f1db0*proj_f1x);
  proj_f1y = proj_f1db0*proj_f1x + proj_f1db1*proj_f1x_1 + proj_f1db2*proj_f1x_2 - proj_f1da1*proj_f1y_1 - proj_f1da2*proj_f1y_2;
  //proj_f1y = proj_f1y/proj_f1da0;
  
  proj_f1x_2 = proj_f1x_1;
  proj_f1x_1 = proj_f1x;
  
  proj_f1y_2 = proj_f1y_1;
  proj_f1y_1 = proj_f1y;
  /*Fim - Filtro de setPoint 1*/

  //Serial.print(proj_f1y);
  //Serial.print("-");

  /*Início - Filtro de setPoint 2*/
  proj_f2x = proj_f1y;
  proj_f2y = proj_f2db0*proj_f2x + proj_f2db1*proj_f2x_1 + proj_f2db2*proj_f2x_2 - proj_f2da1*proj_f2y_1 - proj_f2da2*proj_f2y_2;
  //proj_f2y = proj_f2y/proj_f2da0;
  
  proj_f2x_2 = proj_f2x_1;
  proj_f2x_1 = proj_f2x;
  
  proj_f2y_2 = proj_f2y_1;
  proj_f2y_1 = proj_f2y;
  /*Fim - Filtro de setPoint 2*/

  //Serial.print(proj_f2y);
  //Serial.print("-");

  //Entrada do Controlador - Sinal de Erro
  proj_x = proj_f2y - proj_y;

  /* Início - Controlador Proporcional*/
  proj_px = proj_x;
  proj_py = proj_kp*proj_px;
  /* Fim - Controlador Proporcional */

  //Serial.print(proj_py);
  //Serial.print("-");

  /*Início - Controlador Integral*/
  proj_ix = proj_x;
  proj_ix = proj_ix*proj_ki;
  proj_ix = proj_ix + (proj_invTt * (proj_u-proj_uNs));
  proj_iy = proj_idb0*proj_ix + proj_idb1*proj_ix_1 - proj_ida1*proj_iy_1;
  //proj_iy = proj_iy/proj_ida0;
  proj_ix_1 = proj_ix;
  proj_iy_1 = proj_iy;
  /*Fim - Controlador Integral*/

  //Serial.print(proj_iy);
  //Serial.print("-");
  
  /*Início - Controlador Derivativo*/
  proj_dx = proj_x;
  proj_dx = proj_kd*proj_dx;
  proj_dy = proj_ddb0*proj_dx + proj_ddb1*proj_dx_1 - proj_dda1*proj_dy_1;
  //proj_dy = proj_dy/proj_dda0;
  proj_dx_1 = proj_dx;
  proj_dy_1 = proj_dy;
  /*Fim - Controlador Derivativo*/

  //Serial.print(proj_dy);
  //Serial.print("-");
  
  //Saída do Controlador - Sinal de Controle
  proj_u = proj_py+proj_iy+proj_dy;
  //Saída do controlador Não Saturada
  proj_uNs = proj_u;

  //Saturação do Sinal de Controle (-30 a 30 graus)
  if(proj_u >= 300)
    proj_u = 300;
  else if(proj_u <= -300)
    proj_u = -300;

  //Serial.println(proj_u);
} 
/* FIM - PROCEDIMENTO COMPUTAR COEFICIENTES DO CONTROLADOR DE MODELAGEM E PROJETO DE CONTROLADOR */

/* INÍCIO - PROCEDIMENTO SET COEFICIENTES DOS PARÂMETROS DO MÓDULO DE MODELAGEM E PROJETO DE CONTROLADOR */
void proj_set_parametros() {
  //Parâmetros do Controlador
  proj_lambdaC = proj_const_lambdaC * proj_teta;
  proj_lambdaF = proj_const_lambdaF * proj_lambdaC;
  proj_tf = (pow(proj_lambdaC,4))/((4*(pow(proj_lambdaC,3)))+(6*pow(proj_lambdaC,2)*proj_teta)+(4*proj_lambdaC*pow(proj_teta,2))+(pow(proj_teta,3)));
  proj_ti = (4*proj_lambdaC)+proj_teta-proj_tf;
  proj_td = (((6*pow(proj_lambdaC,2))+(4*proj_lambdaC*proj_teta)+(pow(proj_teta,2)))/proj_ti)-proj_tf;
  proj_kc = (proj_ti/proj_kPlanta)/((4*(pow(proj_lambdaC,3)))+(6*pow(proj_lambdaC,2)*proj_teta)+(4*proj_lambdaC*pow(proj_teta,2))+(pow(proj_teta,3)));

  //Parâmetros do Filtro de SetPoint 1 Contínuo
  proj_f1cb0 = pow(proj_lambdaC,2);
  proj_f1cb1 = 2*proj_lambdaC;
  proj_f1cb2 = 1;
  proj_f1ca0 = (6*(pow(proj_lambdaC,2)))+(4*proj_lambdaC*proj_teta)+(pow(proj_teta,2));
  proj_f1ca1 = (4*proj_lambdaC)+proj_teta;
  proj_f1ca2 = 1;

  //Coeficientes do Filtro de SetPoint 1 Discreto
  //Numerador - Potências de Z Crescente
  proj_f1db0 = (4 * proj_f1cb0) + (2 * proj_Ts_segundos * proj_f1cb1) + (proj_f1cb2 * pow(proj_Ts_segundos, 2));
  proj_f1db1 = (-8 * proj_f1cb0) + (2 * proj_f1cb2 * pow(proj_Ts_segundos, 2));
  proj_f1db2 = (4 * proj_f1cb0) - (2 * proj_Ts_segundos * proj_f1cb1) + (proj_f1cb2 * pow(proj_Ts_segundos, 2));
  //Denominador - Potências de Z Crescente
  proj_f1da0 = (4 * proj_f1ca0) + (2 * proj_Ts_segundos * proj_f1ca1) + (proj_f1ca2 * pow(proj_Ts_segundos, 2));
  proj_f1da1 = (-8 * proj_f1ca0) + (2 * proj_f1ca2 * pow(proj_Ts_segundos, 2));
  proj_f1da2 = (4 * proj_f1ca0) - (2 * proj_Ts_segundos * proj_f1ca1) + (proj_f1ca2 * pow(proj_Ts_segundos, 2));

  //Normaliza Coeficientes do Filtro de SetPoint Discreto 1
  proj_f1db0 = proj_f1db0/proj_f1da0;
  proj_f1db1 = proj_f1db1/proj_f1da0;
  proj_f1db2 = proj_f1db2/proj_f1da0;
    
  proj_f1da1 = proj_f1da1/proj_f1da0;
  proj_f1da2 = proj_f1da2/proj_f1da0;
  proj_f1da0 = 1;
    
  //Parâmetros do Filtro de SetPoint 2 Contínuo
  proj_f2cb0 = pow(proj_lambdaC,2);
  proj_f2cb1 = 2*proj_lambdaC;
  proj_f2cb2 = 1;

  proj_f2ca0 = pow(proj_lambdaF,2);
  proj_f2ca1 = 2*proj_lambdaF;
  proj_f2ca2 = 1;

  //Coeficientes do Filtro de SetPoint 2
  //Numerador - Potências de Z Crescente
  proj_f2db0 = (4 * proj_f2cb0) + (2 * proj_Ts_segundos * proj_f2cb1) + (proj_f2cb2 * pow(proj_Ts_segundos, 2));
  proj_f2db1 = (-8 * proj_f2cb0) + (2 * proj_f1cb2 * pow(proj_Ts_segundos, 2));
  proj_f2db2 = (4 * proj_f2cb0) - (2 * proj_Ts_segundos * proj_f2cb1) + (proj_f2cb2 * pow(proj_Ts_segundos, 2));
  //Denominador - Potências de Z Crescente
  proj_f2da0 = (4 * proj_f2ca0) + (2 * proj_Ts_segundos * proj_f2ca1) + (proj_f2ca2 * pow(proj_Ts_segundos, 2));
  proj_f2da1 = (-8 * proj_f2ca0) + (2 * proj_f2ca2 * pow(proj_Ts_segundos, 2));
  proj_f2da2 = (4 * proj_f2ca0) - (2 * proj_Ts_segundos * proj_f2ca1) + (proj_f2ca2 * pow(proj_Ts_segundos, 2));

  //Normaliza Coeficientes do Filtro de SetPoint Discreto 2
  proj_f2db0 = proj_f2db0/proj_f2da0;
  proj_f2db1 = proj_f2db1/proj_f2da0;
  proj_f2db2 = proj_f2db2/proj_f2da0;
    
  proj_f2da1 = proj_f2da1/proj_f2da0;
  proj_f2da2 = proj_f2da2/proj_f2da0;
  proj_f2da0 = 1;
  
  //Coeficientes do Controlador Integral
  //Numerador - Potências de Z Crescente
  proj_idb0 = 0;
  proj_idb1 = proj_Ts_segundos;
  //Denominador - Potências de Z Crescente
  proj_ida0 = 1.0;
  proj_ida1 = -1.0;

  //Normaliza Parâmetros do Termo Integral
  proj_idb0 = proj_idb0/proj_ida0;
  proj_idb1 = proj_idb1/proj_ida0;
  proj_ida1 = proj_ida1/proj_ida0;
  proj_ida0 = 1.0;
  
  //Coeficientes do Controlador Derivativo com Filtro
  //Numerador - Potências de Z Crescente
  proj_ddb0 = 1;
  proj_ddb1 = -1;
  //Denominador - Potências de Z Crescente
  proj_dda0 = proj_tf+proj_Ts_segundos;
  proj_dda1 = -proj_tf;

  //Normaliza Parâmetros do Termo Derivativo
  proj_ddb0 = proj_ddb0/proj_dda0;
  proj_ddb1 = proj_ddb1/proj_dda0;
  proj_dda1 = proj_dda1/proj_dda0;
  proj_dda0 = 1;
  
  //Ganhos dos Controladores
  proj_kp = proj_kc;
  proj_ki = proj_kc/proj_ti;
  proj_kd = proj_kc*proj_td;

  //Variável que Armazena o Inverso do Tempo Reset do Controlador Integral
  proj_invTt = 1/sqrt(proj_td*proj_ti);


//Caracter Indicativo de Leituras no terminal 
        Serial.println("working...");

//ocorre apenas quando DEBUG esta ativado 
#ifdef DEBUG
    Serial.print("proj_lambdaC = ");
    Serial.println(proj_lambdaC);
    Serial.print("proj_lambdaF = ");
    Serial.println(proj_lambdaF);
    Serial.print("proj_ti = ");
    Serial.println(proj_ti);
    Serial.print("proj_td = ");
    Serial.println(proj_td);
    Serial.print("proj_kc = ");
    Serial.println(proj_kc);
    Serial.println();
    Serial.print("proj_f1cb0 = ");
    Serial.println(proj_f1cb0);
    Serial.print("proj_f1cb1 = ");
    Serial.println(proj_f1cb1);
    Serial.print("proj_f1cb2 = ");
    Serial.println(proj_f1cb2);
    Serial.println();
    Serial.print("proj_f1ca0 = ");
    Serial.println(proj_f1ca0);
    Serial.print("proj_f1ca1 = ");
    Serial.println(proj_f1ca1);
    Serial.print("proj_f1ca2 = ");
    Serial.println(proj_f1ca2);
    Serial.println();
    Serial.print("proj_f1db0 = ");
    Serial.println(proj_f1db0);
    Serial.print("proj_f1db1 = ");
    Serial.println(proj_f1db1);
    Serial.print("proj_f1db2 = ");
    Serial.println(proj_f1db2);
    Serial.println();
    Serial.print("proj_f1da0 = ");
    Serial.println(proj_f1da0);
    Serial.print("proj_f1da1 = ");
    Serial.println(proj_f1da1);
    Serial.print("proj_f1da2 = ");
    Serial.println(proj_f1da2);
    Serial.println();
    Serial.print("proj_f2cb0 = ");
    Serial.println(proj_f2cb0);
    Serial.print("proj_f2cb1 = ");
    Serial.println(proj_f2cb1);
    Serial.print("proj_f2cb2 = ");
    Serial.println(proj_f2cb2);
    Serial.println();
    Serial.print("proj_f2ca0 = ");
    Serial.println(proj_f2ca0);
    Serial.print("proj_f2ca1 = ");
    Serial.println(proj_f2ca1);
    Serial.print("proj_f2ca2 = ");
    Serial.println(proj_f2ca2);
    Serial.println();
    Serial.print("proj_f2db0 = ");
    Serial.println(proj_f2db0);
    Serial.print("proj_f2db1 = ");
    Serial.println(proj_f2db1);
    Serial.print("proj_f2db2 = ");
    Serial.println(proj_f2db2);
    Serial.println();
    Serial.print("proj_f2da0 = ");
    Serial.println(proj_f2da0);
    Serial.print("proj_f2da1 = ");
    Serial.println(proj_f2da1);
    Serial.print("proj_f2da2 = ");
    Serial.println(proj_f2da2);
    Serial.println();
    Serial.print("proj_idb0 = ");
    Serial.println(proj_idb0);
    Serial.print("proj_idb1 = ");
    Serial.println(proj_idb1);
    Serial.println();
    Serial.print("proj_ida0 = ");
    Serial.println(proj_ida0);
    Serial.print("proj_ida1 = ");
    Serial.println(proj_ida1);
    Serial.println();
    Serial.print("proj_ddb0 = ");
    Serial.println(proj_ddb0);
    Serial.print("proj_ddb1 = ");
    Serial.println(proj_ddb1);
    Serial.println();
    Serial.print("proj_dda0 = ");
    Serial.println(proj_dda0);
    Serial.print("proj_dda1 = ");
    Serial.println(proj_dda1);
    Serial.println();
    Serial.print("proj_kp = ");
    Serial.println(proj_kp);
    Serial.print("proj_ki = ");
    Serial.println(proj_ki);
    Serial.print("proj_kd = ");
    Serial.println(proj_kd);
    Serial.println();
    Serial.print("proj_invTt = ");
    Serial.println(proj_invTt);
#endif

}
/* FIM - PROCEDIMENTO SET COEFICIENTES DOS PARÂMETROS DO MÓDULO DE MODELAGEM E PROJETO DE CONTROLADOR */

