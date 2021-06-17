// Код собран из библиотек 
#include <PID_v1.h> // ПИДРегулятор brettbeauregard.com
#include <PID_AutoTune_v0.h> //Автонастройка brettbeauregard.com
#include <Adafruit_MAX31855.h>//Библиотека производителя оригинальных чипов
#include <EEPROM.h> // для хранения коэффициентов в энергонезависимой памяти
#include <LiquidCrystal_I2C_Menu.h> // экран с меню модифицированная каким-то хмырём дописано управление с энкодера с кнопкой
//#include <SPI.h>
#include <PWMrelay.h>//счетчик для реле создающий медленный шим-сигнал

// max31855 пины
#define MAXDO   4
#define MAXCS   5
#define MAXCLK  6

//запись в EEPROM
double kp = EEPROM.get(0, kp);
double ki = EEPROM.get(4, ki);
double kd = EEPROM.get(8, kd);

//дефайны записываются в flash память при компиляции: э-экономия!
#define pinCLK 10
#define pinDT 11
#define pinSW 12

#define pwmPin 9 // нагреватель
#define ventPin 15 // вентилятор  
#define valvePin 16 // заслонка
#define readyPin 17 // светодиод
#define eStop 2 // Аварийная остановка

//Переменные для библиотек настройки и регулятора
int cycleTime = 2;
double setpoint = 100;
double input, output; //входящая температура, выходной сигнал реле(0-255)

//инициализация библиотек
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
PWMrelay relay(pwmPin, HIGH, 2000);
LiquidCrystal_I2C_Menu lcd(0x27, 20, 4);
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);

// Флаги  -------------------------------->> расписать флаги и таймеры
bool preheat = false;
bool tempChecking = false;
bool tuning = false;
bool runing = false;
bool workingFlag = false;
bool postFlag = false;
bool stopped = true;
bool valve = true;
bool mainMenuFlag = true;
bool setupMenuFlag = false;

//Таймеры
uint32_t tmr = 0;
uint32_t workingTimer = 0;
uint32_t tempCheckingTimer = 0;
uint32_t tempMenuUpdateTimer = 0;
unsigned long lastActionTime;
int8_t prevTemp;
uint8_t remainTime;
//uint32_t initTime;
uint16_t backLightTime = 60;
//unsigned long counter;

// Структура для перечисление значений, используемых в меню для задания связи родитель-потомок
enum {mkBack, 
      mkRoot, 
      mkRun, 
      mkStop, 
      mkStatus, 
      mkSetTemperature, 
      mkSetCycleTime, 
      mkSetAutotune, 
      mkSetBackward, 
      mkKp, 
      mkKi, 
      mkKd};

// Описание меню на дисплее 2004
sMenuItem menu[] = {
  {mkBack, mkRoot, "Setup menu"},
  {mkRoot, mkStatus, NULL},
  {mkRoot, mkRun, NULL},
  {mkRoot, mkStop, NULL},
  {mkRoot, mkSetTemperature, NULL},
  {mkRoot, mkSetCycleTime, NULL},
  {mkRoot, mkSetAutotune, NULL},
  
  {mkRoot, mkKp, NULL},
  {mkRoot, mkKi, NULL},
  {mkRoot, mkKd, NULL}

};

// Определение количества элементов меню
uint8_t menuLen = sizeof(menu) / sizeof(sMenuItem);

// Функция поиска индекса элемента меню по его значению key
int getItemIndexByKey(uint8_t key) {
  for (uint8_t i = 0; i < menuLen; i++)
    if (menu[i].key == key)
      return i;
  return -1;
}

// Функция формирования названия пункта меню со значением
void updateCaption(uint8_t key, char format[], int value) {
  // key - ключ обновляемого пункта меню
  // format - шаблон для сборки названия пункта меню
  // value - значение, добавляемое в название
  uint8_t index = getItemIndexByKey(key);
  char* buf = (char*)malloc(40);
  sprintf(buf, format, value);
  menu[index].caption = (char*)realloc(menu[index].caption, strlen(buf) + 1);
  strcpy(menu[index].caption, buf);
  free(buf);
}

void showMainMenu() {
  //тут же необходимо прикрутить отключение подсветки
  // + переодический опрос датчика с выводом на экран
  //long counter = millis();

  //mainMenuFlag = true; // использовать флаг для отсчета оставшегося времени
  //lcd.clear();
  lcd.printAt(0, 0, "Temp: ");
  lcd.printAt(7, 0, input);
  /*
  if (millis() - counter > 1000) {
    lcd.printAt(7, 0, input);
    counter = millis();
  }
  */
  lcd.printAt(0, 1, "Setpoint: ");
  lcd.printAt(10, 1, setpoint);

  lcd.printAt(0, 2, "Cycle time: ");
  lcd.printAt(12, 2, cycleTime);
    
  lcd.printAt(0, 3, "O: ");
  lcd.printAt(4, 3, output);
  
  if (workingFlag) {
    lcd.printAt(15, 2, "/");
    lcd.printAt(17, 2, remainTime);
  } else lcd.printAt(15, 2, "     ");
}

float stableResult() {

  float res = thermocouple.readCelsius();
  float prevResult;
  if (!isnan(res)) {
    prevResult = res;
    return res;
  }
  else {
    //Serial.println(" **************** NaN ************** ");
    return prevResult;
  }
}

// *************************  S E T U P *********************
void setup()
{
  //Serial.begin(9600);
  lcd.begin();
  lcd.attachEncoder(pinDT, pinCLK, pinSW);
    
  updateCaption(mkStatus, "Status", NULL);
  updateCaption(mkRun, "Run", NULL);
  updateCaption(mkStop, "Stop", NULL);
  updateCaption(mkSetTemperature, "Temp: %d C", int(setpoint));
  updateCaption(mkSetCycleTime, "Time: %d min", cycleTime);
  updateCaption(mkSetAutotune, "Autotune", NULL);
  
// Нужна ли проверка полученного из eeprom на !=255 ?
  updateCaption(mkKp, "Kp ", EEPROM.get(0, kp));
  updateCaption(mkKi, "Ki ", EEPROM.get(4, ki));
  updateCaption(mkKd, "Kd ", EEPROM.get(8, kd));

  //uint8_t selectedMenuItem = lcd.showMenu(menu, menuLen, 1);

  delay(500);
  input = stableResult();
  pinMode(valvePin, OUTPUT);
  pinMode(ventPin, OUTPUT);
  pinMode(eStop, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(readyPin, OUTPUT);
  aTune.SetControlType(1);
  myPID.SetMode(AUTOMATIC);
  
  digitalWrite(valvePin, HIGH); // закрыть заслонку на старте  
  digitalWrite(readyPin, HIGH);
  showMainMenu();
  
}




/*
 *    **************    Блок функций    ***************
 * 
 */

void showSetupMenu(){
  
    uint8_t selectedMenuItem = lcd.showMenu(menu, menuLen, 1);
    lastActionTime = millis();
/*
    if(millis() - lastActionTime >= 1000){
      showMainMenu();
      setupMenuFlag = false;
      mainMenuFlag = true;
    }
*/
// Старт рабочего процесса
    if (selectedMenuItem == mkRun) {
      lcd.backlight();
      lastActionTime = millis();
      showMainMenu();

      //setupMenuFlag = false;
      //mainMenuFlag = true;

      runing = true;
      digitalWrite(ventPin, HIGH);
      digitalWrite(valvePin, HIGH);
      digitalWrite(readyPin, LOW);
      lcd.printAt(13, 3, "Run");     
    }

// Полная остановка нагрева с продувкой и последующим выключением вентиляторов
    else if (selectedMenuItem == mkStop) {
      lcd.backlight();
      showMainMenu();
      //setupMenuFlag = false;
      //mainMenuFlag = true;
      lcd.printAt(13, 3, "Stp");
      postFlag = true;
      lastActionTime = millis();
      
    }

// Задание температуры (Вероятно, что нужно хранить в EEPROM)
    else if (selectedMenuItem == mkSetTemperature) {
      setpoint = lcd.inputVal("Setup Temperature", 10, 250, int(setpoint));
      updateCaption(mkSetTemperature, "Temp: %d C", int(setpoint));
      lcd.backlight();
      lastActionTime = millis();
    }

// Задание времени рабочего процесса (Вероятно, что нужно хранить в EEPROM)
    else if (selectedMenuItem == mkSetCycleTime) {
      cycleTime = lcd.inputVal("Setup Cycle Time", 0, 20, cycleTime);
      updateCaption(mkSetCycleTime, "Time: %dmin", cycleTime);
      lcd.backlight();
      lastActionTime = millis();
    }

// Автонастройка коэффициентов 
    else if (selectedMenuItem == mkSetAutotune) {
      showMainMenu();
      //setupMenuFlag = false;
      //mainMenuFlag = true;
      //runAutotuning();
      preheat = true;
      output = 255;
      lcd.backlight();
      lastActionTime = millis();
      //Serial.println("Preheat is started");
      lcd.printAt(13, 3, "A-tune");

      digitalWrite(ventPin, HIGH); // проверить состояние клапана и закрыть
      digitalWrite(readyPin, LOW);
    }

// Редактирование коэффициентов
    else if (selectedMenuItem == mkKp) {
      lcd.inputValBitwise("Enter Kp: ", kp, 5, 2, 0);
      EEPROM.put(0, kp);
      lcd.backlight();
      //lastActionTimer = millis();
    }
    else if (selectedMenuItem == mkKi) {
      lcd.inputValBitwise("Enter Ki: ", ki, 5, 2, 0);
      EEPROM.put(4, ki);
      lcd.backlight();
      //lastActionTimer = millis();
    }
    else if (selectedMenuItem == mkKd) {
      lcd.inputValBitwise("Enter Kd: ", kd, 5, 2, 0);
      EEPROM.put(8, kd);
      lcd.backlight();
      //lastActionTimer = millis();
    }
// Показать статус
    else if (selectedMenuItem == mkStatus) {
      showMainMenu();
      //setupMenuFlag = false;
      //mainMenuFlag = true;
      lcd.backlight();
      //lastActionTimer = millis();
    }
}

void runingWorkingProcess() {
  myPID.Compute();
  remainTime = (cycleTime - ((millis()/60000) - (workingTimer/60000))); //осталось минут рабочего процесса

  if (input >= setpoint && !workingFlag) {
    workingTimer = millis();
    workingFlag = true;  
  }
  if ((millis() - workingTimer) > (cycleTime * 60000) && workingFlag) {
    postFlag = true;
    //tone(eStop, 200, 500);
    workingFlag = false;
    lcd.printAt(13, 3, "End");
    //postFlag = true;
  }
}

void stop_workingProcess() {
  preheat = false;
  tuning = false;
  output = 0;
  runing = false;
  postFlag = true;
  if(input>70){  // ---> ---> --->      условие повторного включения
    digitalWrite(valvePin, LOW);
    digitalWrite(ventPin, HIGH);
  }
  else {
    digitalWrite(valvePin, HIGH);
    digitalWrite(ventPin, LOW);
    digitalWrite(readyPin, HIGH);
    postFlag = false;
    //пищалку включить
  }  
}

void preheating() {
  //  При достижении заданной температуры выключить нагрев +
  // проверка стабилизации температуры
  // инициализация переменных необходимых для проверки.

  if (input >= (setpoint-2) && preheat) {
    output = 0;
    relay.setPWM(output);
    relay.tick();

    //analogWrite(pwmPin, output);
    preheat = false;
    //Serial.println("Preheat is stopped");
    tempChecking = true;
    tempCheckingTimer = millis();
    prevTemp = input;
  }


}

void autoTuning() {

  byte val = (aTune.Runtime());
  if (val != 0) {
    tuning = false;
    //postFlag = true;
    postFlag = true;

    EEPROM.put(0, aTune.GetKp());
    EEPROM.put(4, aTune.GetKi());
    EEPROM.put(8, aTune.GetKd());

    output = 0;
    showMainMenu();
    lcd.printAt(13, 3, "StpTune");
    //tone(eStop, 500, 1500);

    /*
      lcd.printAt(0, 0, EEPROM.get(0, kp));
      lcd.printAt(0, 1, EEPROM.get(4, ki));
      lcd.printAt(0, 2, EEPROM.get(8, kd));

      Serial.println("End of Runtime");
      Serial.print("Kp = ");
      Serial.print(kp);
      Serial.print("Ki = ");
      Serial.print(ki);
      Serial.print("Kd = ");
      Serial.println(kd);
    */
  }
}

void tempCheck() {

  if (millis() - tempCheckingTimer > 5000 && input <= prevTemp && tempChecking) {

    byte ATuneModeRemember = 1;
    byte aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 50; // проверку на output_Max>255 
    byte aTuneLookBack = 3; // период определения мин и макс раскачки (sec)

    tempCheckingTimer = millis();
    //Serial.println("Calibration");
    tempChecking = false;
    tuning = true;

    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    output = aTuneStartValue;
    //ATuneModeRemember = myPID.GetMode();
    //tempCheckingTimer = 0;

    relay.setPWM(output);
    relay.tick();
    //analogWrite(pwmPin, output);
  }
  else if (millis() - tempCheckingTimer > 5000 && input > prevTemp && tempChecking) {
    tempCheckingTimer = millis();
    //Serial.println("Temperature checking");
    prevTemp = input;
  }

}
/*
float stableResult() {

  float res = thermocouple.readCelsius();
  float prevResult;
  if (!isnan(res)) {
    prevResult = res;
    return res;
  }
  else {
    //Serial.println(" **************** NaN ************** ");
    return prevResult;
  }
}
*/
void backlightControl() {
  if (millis() - lastActionTime > (backLightTime*1000) && mainMenuFlag) {
    lcd.noBacklight();
  }
}



// *************************  L O O P *********************

void loop() {

//  backlightControl();
  input = stableResult();
  relay.setPWM(output);
  relay.tick();

// Условие для обновления темп на гл экране (И обновлении времени при запуске раб. процесса?)
  if (millis() - tempMenuUpdateTimer > 1000 && mainMenuFlag) {
    tempMenuUpdateTimer = millis();
    showMainMenu();
  }

  //if(input > (setpoint+20) || 


// Обработка нажатия кнопки энкодера
  if (lcd.getEncoderState() == eButton) {
    //uint8_t selectedMenuItem = lcd.showMenu(menu, menuLen, 1);
    lcd.backlight();
    showSetupMenu();
    //setupMenuFlag = true;
    //mainMenuFlag = false;
  }
/*  
  if(millis() - lastActionTime >= 1000){
    showMainMenu();
    setupMenuFlag = false;
    mainMenuFlag = true;
  }
*/
// Запуск рабочего процесса
  if (runing) {
    runingWorkingProcess();
  }
// Преднагрев
  if (preheat) {
    preheating();
  }
// Проверка температуры и включение автонастройки
  if (tempChecking) {
    tempCheck();
  }
// Старт автонастройки
  if (tuning) {
    autoTuning();
  }
  
  
  if (postFlag) {
    stop_workingProcess();
  }

    /* В случае ошибки:
     * Выключить нагрев, остановить вентилятор, моргать своей лампочкой и пищать
     * Перегрев на 20-30 градусов выше уставки
     * недостаточная скорость нагрева(измерить полный нагрев с холодного до уставки добавить 15-30%)
     * срабатывание кнопки аварийной остановки
     * 
     */

  /*
  if(mainMenuFlag){
    showMainMenu();
  }
  else if(setupMenuFlag){
    showSetupMenu();
  }
  
  
    // вывод в порт
    if(millis()-tmr > 500){
      tmr = millis();
      Serial.print(setpoint);
      Serial.print(" ");
      Serial.print(input);
      Serial.print(" ");
      Serial.print(output);
      Serial.println(" ");
    }
  */
}
