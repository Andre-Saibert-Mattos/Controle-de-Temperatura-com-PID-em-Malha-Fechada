#include <LiquidCrystal_I2C.h>    // Bibliioteca para o uso do LCD com I2C
LiquidCrystal_I2C lcd(0x27,20,4); // Defina o endereço LCD para 0x27 para um display de 16 caracteres e 2 linhas

class PID{
  public:
  double error;
  double sample;
  double lastSample;
  double kP, kI, kD;
  double P, I, D;
  double pid;
  double setPoint;
  long lastProcess;
  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  void addNewSample(double _sample){
    sample = _sample;
  }
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }
  double process(){
    // Implementação P ID
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    //P
    P = error * kP;
    //I
    I += ((error * kI) * deltaTime)/3.125;
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    // Soma tudo
    pid = P + I + D;
    return pid;
  }
};
PID meuPid(700, 20, 5);

#define pin_lm35 4 //Pino o qual o sensor está conectado
#define pin_POT_Controle 15
#define pin_PWM 14  // 16 corresponds to GPIO16
#define pin_POT_Disturbio 25
#define pin_Disturbio 26

#define Temperatura_Max 45.00
#define Temperatura_Min 25.00

void leitura_Temperatura();

// Set propriedades do PWM
const int freq = 1000;
const int canal_PWM = 0;
const int canal_Disturbio = 1;
const int resolution = 12;  // Em bits

float temperatura_soma = 0.0;
float temperatura_Atual = 0.0;
float maps_Entrada_POT = 0.0;
int controlePwm = 0;
int controleDisturbio = 0;
double erro=0;

int aux=0;
int aux2=0;

bool ler_Temperatura = false;

void interrupcao_timer(){
  ler_Temperatura=true;
}

void setup() {
  Serial.begin(115200);
  lcd.init();         // Inicia o LCD 
  lcd.backlight();    // Liga a Luz de fundo
  
  pinMode(pin_lm35, INPUT);
  pinMode(pin_POT_Controle, INPUT);
  pinMode(pin_POT_Disturbio, INPUT);

  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_Disturbio, OUTPUT);
  
  ledcSetup(canal_PWM, freq, resolution); // Configura o PWM para o controle da resistencia
  ledcAttachPin(pin_PWM, canal_PWM);      // Vincula o canal ao GPIO para ser controlado

  ledcSetup(canal_Disturbio, freq, resolution); // Configura o PWM para o disturbio
  ledcAttachPin(pin_Disturbio, canal_Disturbio);      // Vincula o canal ao GPIO para ser controlado
  
  // Configurando o Timer2
  hw_timer_t * timer = NULL;
  timer = timerBegin(2, 80, true);
  timerAttachInterrupt(timer, &interrupcao_timer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
}
 
void loop() {
  if(ler_Temperatura){
    ler_Temperatura=false;
    
    maps_Entrada_POT = map(float(analogRead(pin_POT_Controle)), 0, 4095, Temperatura_Min, Temperatura_Max);

    controleDisturbio = float(analogRead(pin_POT_Disturbio));
    ledcWrite(canal_Disturbio, controleDisturbio);

    leitura_Temperatura();

    if(aux2==32){  
      
      lcd.setCursor(0,0);  
      lcd.print("Temp Att.: ");
      lcd.setCursor(11,0);
      lcd.print("     ");
      lcd.setCursor(11,0);
      lcd.print(temperatura_Atual);
      lcd.setCursor(0,1);
      lcd.print("Temp Def.: ");
      lcd.setCursor(11,1);  
      lcd.print("      ");
      lcd.setCursor(11,1);
      lcd.print(maps_Entrada_POT);
      
    }else if(aux2==12){
      Serial.print(temperatura_Atual);
      Serial.print(" ");
      Serial.print(maps_Entrada_POT);
      Serial.print(" ");
      erro = maps_Entrada_POT - temperatura_Atual;
      if(erro<0)erro=0;
      Serial.print(erro);
      Serial.print(" ");
      Serial.print(map(controlePwm, 0, 4095, 0, 100));
      Serial.print(" ");
      Serial.println(map(controleDisturbio, 0, 4095, 0, 100));
    }else if(aux2>=32){
      aux2=0;
    }
    aux2++;
  }
}

void leitura_Temperatura(){ 
  if(aux<32){
    aux++;
    temperatura_soma += (((float(analogRead(pin_lm35)) * 5)/4095.0) / 0.010);

    meuPid.addNewSample(temperatura_Atual);
    meuPid.setSetPoint(maps_Entrada_POT);

    controlePwm = meuPid.process();

    if(controlePwm<0)
      controlePwm = 0;
    else if(controlePwm>4095)
      controlePwm = 4095;
      
    ledcWrite(canal_PWM, controlePwm);
  }else{
    temperatura_Atual=0.0;
    temperatura_Atual = temperatura_soma/32;
    temperatura_soma=0.0;
    aux=0;
  }
}
