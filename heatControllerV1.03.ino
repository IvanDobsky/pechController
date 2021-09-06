// Код собран на основе библиотек 
#include <PID_v1.h> // ПИДРегулятор brettbeauregard.com
#include <PID_AutoTune_v0.h> //Автонастройка brettbeauregard.com
#include <Adafruit_MAX31855.h>//Библиотека производителя оригинальных чипов
#include <EEPROM.h> // для хранения коэффициентов и некоторых данных в энергонезависимой памяти. Перед первой прошивкой необходимо в ячейки 0,4,8,20,24,28,32,36 занести данные.
#include <LiquidCrystal_I2C_Menu.h> // экран с меню модифицированная Tsybrov дописано управление с энкодера с кнопкой
#include <PWMrelay.h>//счетчик для реле создающий медленный шим-сигнал

// max31855 пины датчика температуры
#define MAXDO   4
#define MAXCS   5
#define MAXCLK  6

//запись коэффициентов в EEPROM
double kp = EEPROM.get(0, kp);
double ki = EEPROM.get(4, ki);
double kd = EEPROM.get(8, kd);

uint8_t errorCode = EEPROM.get(20, errorCode); // код ошибки, который будет записываться в ЕЕПРОМ


//Пины ардуины. Дефайны записываются в flash память при компиляции для экономии динамической памяти

#define pinCLK 12 //вращение энкодера по часовой
#define pinDT 11 //вращение энкодера против часовой
#define pinSW 10 // пин кнопки

#define pwmPin 9 // нагреватель (твердотельное реле)

#define ventPin 14 // реле вентилятор 
#define valvePin 16 // реле заслонки
#define readyLed 17 // зеленый светодиод
#define errorLed 8 // красный светодиод
#define eStop 2 // кнопка аварийной остановки
#define uzo 7// реле узо
#define relay7 22 //резерв

//Переменные для библиотек настройки и регулятора
int cycleTime = EEPROM.get(28, cycleTime); // время рабочего цикла
double setpoint = EEPROM.get(24, setpoint); // уставка(заданная температура)
double input, output; //входящая температура, выходной сигнал реле(0-255)

uint8_t temp_prev; //переменная предыдущего измерения температуры
uint32_t Time_prev; //предыдущей временной отсечки

//инициализация библиотек
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
PWMrelay relay(pwmPin, HIGH, 3000);
LiquidCrystal_I2C_Menu lcd(0x27, 20, 4);
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);

// Флаги состояний
//uint8_t statement = 0;// переменная состояния для отображения на главном экране
bool readyFlag = true;
bool preheat = false; //Флаг состояния преднагрева для автонастройки
bool preheatingFlag = false;//Преднагрев для отображения на экране состояния
bool tempChecking = false; //проверка температуры
bool tuning = false; //автонастройка
bool runing = false; //старт рабочего процесса
bool workingFlag = false;//удержание и таймер
bool postFlag = false;//выключение
bool blowingFlag = false; // продувка
bool stopped = true;//отчет об остановке
bool autotuning = false;//состояние заслонки
bool mainMenuFlag = true;//показывает главное меню
bool setupMenuFlag = false;//меню настроек

bool errorFlag = false;//Ошибка
bool e_StopFlag = false;//Нажата кнопка аварийной установки
bool underheatingFlag = false;//недогрев
bool overheatingFlag = false;//перегрев

//Таймеры
//uint32_t tmr = 0;
uint32_t workingTimer = 0;
uint32_t tempCheckingTimer = 0;
uint32_t tempMenuUpdateTimer = 0;
uint32_t lastActionTime;

int temp_user = EEPROM.get(32, temp_user); // Минимальная температура, которая должна набираться за время Time_user
int Time_user = EEPROM.get(36, Time_user); // Максимальное время, за которое должна набираться температура temp_user
double CriticalHysteresis = 10;

//double errTemp; // тестовая переменная
uint32_t timer1 = 0; // Таймер для перепроверки значения при сигнале о перегреве/недогреве уже в процессе запекания.

int8_t prevTemp;
uint8_t remainTime;
//uint32_t initTime;
uint16_t backLightTime = 90;
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
      
      mkSetTemp_Protection, 
      mkSetTime_Protection, 
      
      mkSetBackward, 
      mkKp, 
      mkKi, 
      mkKd,
      mkError_Code};

// Описание меню на дисплее 2004
sMenuItem menu[] = {
  {mkBack, mkRoot, "Setup menu"},
  {mkRoot, mkStatus, NULL},
  {mkRoot, mkRun, NULL},
  {mkRoot, mkStop, NULL},
  {mkRoot, mkSetTemperature, NULL},
  {mkRoot, mkSetCycleTime, NULL},
  {mkRoot, mkSetAutotune, NULL},
  
  {mkRoot, mkSetTemp_Protection, NULL},
  {mkRoot, mkSetTime_Protection, NULL},
  
  {mkRoot, mkKp, NULL},
  {mkRoot, mkKi, NULL},
  {mkRoot, mkKd, NULL},
  {mkRoot, mkError_Code, NULL}

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
  lcd.clear();
  lcd.printAt(0, 0, "Temp: ");
  lcd.printAt(7, 0, int(input));
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

  if(readyFlag)                           lcd.printAt(5, 3, "Ready");
  else if(e_StopFlag)                     lcd.printAt(2, 3, "E-Stop UZO OFF");
  else if(overheatingFlag && errorFlag)   lcd.printAt(1, 3, "Overheat UZO OFF");
  else if(underheatingFlag && errorFlag)  lcd.printAt(3, 3, "Underheating");
  else if(preheatingFlag && !autotuning)  lcd.printAt(4, 3, "Heating...");
  else if(workingFlag)                    lcd.printAt(5, 3, "BAKING...");
  else if(blowingFlag)                    lcd.printAt(5, 3, "Blowing");
  else if(autotuning)                     lcd.printAt(4, 3, "Autotuning...");
  else                                    lcd.printAt(0, 3, "");
  
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

void error(){

  errorFlag = true;
  readyFlag = false;  
  digitalWrite(readyLed, LOW);
  digitalWrite(errorLed, HIGH);
  digitalWrite(ventPin, LOW);
  preheat = false;
  tuning = false;
  output = 0;
  runing = false;
  if(e_StopFlag){
    EEPROM.put(20, 1);
    digitalWrite(uzo, LOW);
  }
  if(underheatingFlag){
    EEPROM.put(20, 2);
  }
  if(overheatingFlag){
    EEPROM.put(20, 3);
    digitalWrite(uzo, LOW);
  }

  // Включить красную лампочку, вывести сообщение об аварийной остановке, причину остановки(перегрев-недогрев-нажата кнопка)
}
// *************************  S E T U P *********************
void setup()
{
  //Serial.begin(9600);.
  lcd.begin();
  lcd.attachEncoder(pinDT, pinCLK, pinSW);
    
  updateCaption(mkStatus, "Status", NULL);
  updateCaption(mkRun, "Run", NULL);
  updateCaption(mkStop, "Stop", NULL);
  updateCaption(mkSetTemperature, "Temp: %d C", int(setpoint));
  updateCaption(mkSetCycleTime, "Time: %d min", cycleTime);
  updateCaption(mkSetAutotune, "Autotune", NULL);

  updateCaption(mkSetTemp_Protection, "Temp_protect: %d C", temp_user);
  updateCaption(mkSetTime_Protection, "Time_protect: %d min", Time_user);

// Нужна ли проверка полученного из eeprom на !=255 ?
  updateCaption(mkKp, "K_p ", EEPROM.get(0, kp));
  updateCaption(mkKi, "K_i ", EEPROM.get(4, ki));
  updateCaption(mkKd, "K_d ", EEPROM.get(8, kd));
  updateCaption(mkError_Code, "Error Code ", EEPROM.get(20, errorCode));

  delay(500);
  input = stableResult();
  pinMode(valvePin, OUTPUT);
  digitalWrite(ventPin, HIGH);
  pinMode(ventPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(readyLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(eStop, INPUT_PULLUP);
  digitalWrite(uzo, HIGH);
  pinMode(uzo, OUTPUT);
  aTune.SetControlType(1);
  myPID.SetMode(AUTOMATIC);
  
  digitalWrite(valvePin, HIGH); // LOW - закрыть(пустить поток по внутреннему контуру) заслонку, HIGH - открыть(выдув наружу) заслонку
  digitalWrite(ventPin, LOW);
  digitalWrite(readyLed, HIGH);
  digitalWrite(errorLed, LOW);
 
  digitalWrite(eStop, HIGH);
  showMainMenu();
  
}

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

      runing = true;
      readyFlag = false;
      digitalWrite(ventPin, HIGH); // Cooler is run
      digitalWrite(valvePin, HIGH);// Valve is close
      digitalWrite(readyLed, LOW);
      
      int temp_prev = input;
      uint32_t Time_prev = millis();

      lcd.clear();
    }

// Полная остановка нагрева с продувкой и последующим выключением вентиляторов
    else if (selectedMenuItem == mkStop) {
      lcd.backlight();
      workingFlag = false;
      preheatingFlag = false;
      showMainMenu();
      Time_prev = 0;
      temp_prev = 0;
      postFlag = true;
      lastActionTime = millis();
      
    }

// Задание температуры (Вероятно, что нужно хранить в EEPROM)
    else if (selectedMenuItem == mkSetTemperature) {
      setpoint = lcd.inputVal("Setup Temperature", 50, 250, int(setpoint)); 
      EEPROM.put(24, setpoint);
      updateCaption(mkSetTemperature, "Temp: %d C", int(setpoint));
      lcd.backlight();
      lastActionTime = millis();
    }

// Задание времени рабочего процесса (Вероятно, что нужно хранить в EEPROM)
    else if (selectedMenuItem == mkSetCycleTime) {
      cycleTime = lcd.inputVal("Setup Cycle Time", 1, 20, cycleTime);
      EEPROM.put(28, cycleTime);
      updateCaption(mkSetCycleTime, "Time: %dmin", cycleTime);
      lcd.backlight();
      lastActionTime = millis();
    }

// Автонастройка коэффициентов 
    else if (selectedMenuItem == mkSetAutotune) {
      showMainMenu();
      readyFlag = false;
      autotuning = true;
      preheat = true;
      output = 255;
      lcd.backlight();
      lastActionTime = millis();
      lcd.clear();
      digitalWrite(ventPin, HIGH); // проверить состояние клапана и закрыть
      digitalWrite(readyLed, LOW);
    }

// Установка температуры защиты от недогрева
    else if (selectedMenuItem == mkSetTemp_Protection) {
      temp_user = lcd.inputVal("Temp protection", 0, 20, temp_user);
      EEPROM.put(32, temp_user);
      updateCaption(mkSetTemp_Protection, "Temp_prot: %d C", temp_user);
      lcd.backlight();
      lastActionTime = millis();
    }
// Установка времени защиты от недогрева      
    else if (selectedMenuItem == mkSetTime_Protection) {
      Time_user = lcd.inputVal("Time protection", 0, 20, Time_user);
      EEPROM.put(36, Time_user);
      updateCaption(mkSetTime_Protection, "Time_prot: %d min", Time_user);
      lcd.backlight();
      lastActionTime = millis();

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
// Хранение и отображение кода предыдущей ошибки
    else if (selectedMenuItem == mkError_Code) {
      lcd.printAt(1, 1, EEPROM.get(20, errorCode));
      delay(1000);// выход по нажатию
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

void runingWorkingProcess() { //Запуск процесса нагрева до уставки и удержания по текущим коэффициентам.
 
  myPID.Compute();
  remainTime = (cycleTime - ((millis()/60000) - (workingTimer/60000))); //осталось минут рабочего процесса


// Условие проверки после сигнала о перегреве или недогреве(на предмет случайных значений), проверяет значение спустя 3 секунды.
  if(overheatingFlag || underheatingFlag){
    if(millis()-timer1 > 3000){
      if(input>(setpoint+CriticalHysteresis) || input<(setpoint-CriticalHysteresis)){
        error();
      }
      else{
        overheatingFlag=false;
        underheatingFlag=false;
        timer1 = 0;
      }
    }
  }

  // Условия защиты от недогрева в процессе выхода на уставку (setpoint).
  if (input < setpoint && !workingFlag){ 
      
      preheatingFlag = true;
      
      // Тут отслеживается скорость нагрева
      if (millis() - Time_prev < Time_user * 60000 && input - temp_prev >= temp_user){ // Время отсчета не вышло и температура превысила заданную ( Ок! ) обновляет температуру и счетчик.
        Time_prev = millis();
        temp_prev = input;
       }
    
       else if (input - temp_prev < temp_user && millis() - Time_prev > Time_user * 60000){ // Время вышло, а температура не достигла заданной пользователем. Необходимо сообщить об ошибке недогрева!
        underheatingFlag = true;
        error(); //stop working process    
       }
  }
// Достиг уставки.
  if (input >= setpoint && !workingFlag) {
    workingTimer = millis();
    workingFlag = true;
    preheatingFlag = false;  
  }

  if(input > (setpoint + CriticalHysteresis) && workingFlag && !overheatingFlag){
    overheatingFlag = true;
    timer1 = millis();
  }  
  
  if(input < (setpoint - CriticalHysteresis) && workingFlag && !underheatingFlag){
    underheatingFlag = true;
    timer1 = millis();
  }

  if ((millis() - workingTimer) > (cycleTime * 60000) && workingFlag) {
    postFlag = true;
    workingFlag = false;
    }

} 

void stop_workingProcess() {
  Time_prev = 0;
  preheat = false;
  tuning = false;
  output = 0;
  runing = false;
  postFlag = true;

  Time_prev = 0;
  temp_prev = 0;
  
   
  if(input>70 && !runing){  // ---> ---> --->      условие повторного включения
    digitalWrite(valvePin, LOW);// open to outside
    digitalWrite(ventPin, HIGH); // Cooler running
    blowingFlag = true;
    readyFlag = false;
  }
  else {
    digitalWrite(valvePin, HIGH); // Valve to inner flow
    digitalWrite(ventPin, LOW); // Cooler Stoped
    digitalWrite(readyLed, HIGH);
    workingFlag = false;
    blowingFlag = false;
    readyFlag = true;
    postFlag = false;
  }  
}

void preheating() {
  if (input >= (setpoint-2) && preheat) {
    output = 0;
    relay.setPWM(output);
    relay.tick();
    preheat = false;
    tempChecking = true;
    tempCheckingTimer = millis();
    prevTemp = input;
  }
}

void autoTuning() {

  byte val = (aTune.Runtime());
  if (val != 0) {
    tuning = false;
    autotuning = false;
    postFlag = true;

    EEPROM.put(0, aTune.GetKp());
    EEPROM.put(4, aTune.GetKi());
    EEPROM.put(8, aTune.GetKd());

    output = 0;
    readyFlag = true;
    showMainMenu();

    /*
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

void tempCheck() { //Функция для автонастройки

  if (millis() - tempCheckingTimer > 5000 && input <= prevTemp && tempChecking) {

    byte ATuneModeRemember = 1;
    byte aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 50; // проверку на output_Max>255 
    byte aTuneLookBack = 3; // период определения мин и макс раскачки (sec)

    tempCheckingTimer = millis();
    tempChecking = false;
    tuning = true;
    autotuning = true;

    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    output = aTuneStartValue;
    
    relay.setPWM(output);
    relay.tick();
    
  }
  else if (millis() - tempCheckingTimer > 5000 && input > prevTemp && tempChecking) {
    tempCheckingTimer = millis();
    prevTemp = input;
  }

}

void backlightControl() {
  if (millis() - lastActionTime > (backLightTime*1000) && mainMenuFlag) {
    lcd.noBacklight();
  }
}



// *************************  L O O P *********************

void loop() {

  backlightControl();
  input = stableResult();
  relay.setPWM(output);
  relay.tick();
  if (digitalRead(eStop) == true){
    e_StopFlag = true;
    error();
  }

// Условие для обновления темп на гл экране (И обновлении времени при запуске раб. процесса?)
  if (millis() - tempMenuUpdateTimer > 1000 && mainMenuFlag) {
    tempMenuUpdateTimer = millis();
    showMainMenu();
  }

  // Обработка нажатия кнопки энкодера
  if (lcd.getEncoderState() == eButton && !errorFlag) {
    analogWrite(pwmPin, LOW);
    lcd.backlight();
    showSetupMenu();
    
  }
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
// Постпроцесс(отключение нагрева, продувка камеры при необходимости)      
  if (postFlag) {
    stop_workingProcess();
  }

}
