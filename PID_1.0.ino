// 0 -> resfriando(MAX)
// 50 -> desligar
// 100 -> aquecendo (MAX)

// Temperatura: A0 0 - 1024
// 0: 0
// 1024: 100
#define pSensor   A0
#define pControle 6

float setPoint, error;
float P, I, D, Kp, Ki, Kd, Pid;
float lastRead = 0;
float lastProcess = 0;
int Read, controlePWM;

void setup(){
  Kp = 1.5;
  Ki = 0.5;
  Kd = 1;
  Serial.begin(9600);
  pinMode(pSensor, INPUT);
  pinMode(pControle, OUTPUT); 
  setPoint = 60;
  Serial.print("Valor Set Point = ");
  Serial.println(setPoint);
    
}

void loop(){
  if (Serial.available()){
   // Leitura da Serial
  Read = map(analogRead(1), 612,1023,0,100);
  
  // Implementação PID
  error = setPoint - Read;
  float deltaTime = (millis() - lastProcess) / 1000; ;
  lastProcess = millis();
  // P
  P = error*Kp;
  
  // I
  I = I + (error * Ki) * (deltaTime/1000);
  
  // D
  D = (lastRead - Read) * Kd / (deltaTime);
  lastRead = Read;
  
  //PID
  Pid = P + I + D;
  
  // Converter PID para Controle
 

  controlePWM = (Pid+30);
  
   if (controlePWM > 255){
    controlePWM = 255;
  }
  if (controlePWM<0){
    controlePWM = 0;
  }

  // Saída do Controle
  analogWrite(pControle, controlePWM);
  Serial.print("P = ");
  Serial.print(P);
  Serial.print(" ");
  Serial.print("I = ");
  Serial.print(I);
  Serial.print(" ");
  Serial.print("D = ");
  Serial.print(D);
  Serial.print(" ");
  Serial.print("Output = ");
  Serial.print(Read);
  Serial.print(" ");
  Serial.print("PWM = ");
  Serial.println(controlePWM);
  delay(100);
}
}
