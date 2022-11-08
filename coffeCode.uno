#include "Wire.h";
#include "LiquidCrystal_I2C.h";
#include <DallasTemperature.h>  //INCLUSÃO DE BIBLIOTECA

#define DS18B20 8  //sensor de temperatura submerso 
#define Sensor1 7 //sensor de nível de agua (baixo)
#define releb 12  //rele da bomba de água 12v
#define relea 13  //rele do aquecedor 12v

#define buttonpin 2  // declara o push button na porta 2
int estado = 0;      // variável para leitura do pushbutton
int guarda_estado;   // variável para armazenar valores do pushbutton

OneWire ourWire(DS18B20);             //CONFIGURA UMA INSTÂNCIA ONEWIRE PARA SE COMUNICAR COM O SENSOR
DallasTemperature sensors(&ourWire);  //BIBLIOTECA DallasTemperature UTILIZA A OneWire

LiquidCrystal_I2C lcd(0x27, 16, 4);

int nivelinicial = 0;

void setup() {

  pinMode(buttonpin, INPUT_PULLUP);  // define o pino do botao como entrada

  Serial.begin(9600);  //INICIALIZA A SERIAL
  sensors.begin();     //INICIA O SENSOR
  delay(1000);
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);

  pinMode(Sensor1, INPUT);
  pinMode(relea, OUTPUT);
  pinMode(releb, OUTPUT);
  digitalWrite(relea, HIGH);
  digitalWrite(releb, LOW);
}

void loop() {
  sensors.requestTemperatures();
  int estado = digitalRead(buttonpin);
  reabastecer();
  lcd.setCursor(0, 1);
  lcd.print("TEMP.:");
  lcd.print(sensors.getTempCByIndex(0));
  lcd.print("*C");
  if (estado == 0) {
    if (digitalRead(sensor1) == HIGH) {
      reabastecer();
      sensors.requestTemperatures();
      lcd.setCursor(0, 0);
      lcd.print("PREPARANDO...");
      digitalWrite(relea, HIGH);
      if (sensors.getTempCByIndex(0) >= 31.01)
        verificaTemp();
    }
  }
  Serial.println(digitalRead(sensor1));
}
void verificaTemp() {

  digitalWrite(releb, HIGH);
  delay(2000);
  digitalWrite(releb, LOW);
  digitalWrite(relea, LOW);
  estado = 1;
}
void reabastecer() {
  if ((sensor1 == 0)) {
    lcd.setCursor(0, 0);
    lcd.print("  ABASTECA   O  ");
    lcd.setCursor(0, 1);
    lcd.print("  RESERVATORIO  ");
    digitalWrite(releb, LOW);
    digitalWrite(relea, LOW);
    estado = 1;
    delay(2000);
  }
}
