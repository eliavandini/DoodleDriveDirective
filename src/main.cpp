#include <Arduino.h>
#include <HardwareSerial.h>
// #include <Serial.h>
#include <DRV8825.h>
unsigned long last_cycle_time = 0;
bool led_13 = true;

// #define Serial Serial

#define led PC13
// HardwareSerial Serial(USART6);
#define MOTOR_STEPS 200 // from datasheet
#define RPM 100

#define WORKING_HEIGHT 0
#define CANVAS_HEIGHT 0
#define CANVAS_X 0
#define cANVAS_Y 0
#define CANVAS_H 0
#define cANVAS_W 0
// #define

// zu korrigieren
// #define DIR1_PIN PD2
// #define STEP1_PIN PC11
// #define ENABLE1_PIN PC12
// #define SLEEP1_PIN PC10
// #define RESET1_PIN PB4

// #define DIR2_PIN PB9
// #define STEP2_PIN PB5
// #define ENABLE2_PIN PB7
// #define SLEEP2_PIN PC1
// #define RESET2_PIN PC2

// #define DIR3_PIN PA3
// #define STEP3_PIN PA1
// #define ENABLE3_PIN PA2
// #define SLEEP3_PIN PA5
// #define RESET3_PIN PA6

// #define DIR4_PIN PB0
// #define STEP4_PIN PC4
// #define ENABLE4_PIN PC5
// #define SLEEP4_PIN PB10
// #define RESET4_PIN PA10

#define START_BYTE 216
#define END_BYTE 228

#define INSTRUCTION_SEND_BUFF 3
#define INSTRUCTION_ABORT 5
#define INSTRUCTION_PAUSE 10
#define INSTRUCTION_RESUME 11
#define INSTRUCTION_RESTART 15
#define INSTRUCTION_PING 150
#define INSTRUCTION_MEMORY_MAP 130
#define INSTRUCTION_RESULT 101
#define INSTRUCTION_STARTED 110
#define INSTRUCTION_PROGESS 102
#define INSTRUCTION_FINISHED 103
#define INSTRUCTION_ERROR 200

#define RXPIN PC7
#define TXPIN PC6

// Serial Serial(RXPIN, TXPIN);
// DRV8825 stepper1(MOTOR_STEPS, DIR1_PIN, STEP1_PIN, ENABLE1_PIN);
// DRV8825 stepper2(MOTOR_STEPS, DIR2_PIN, STEP2_PIN, ENABLE2_PIN);
// DRV8825 stepper3(MOTOR_STEPS, DIR3_PIN, STEP3_PIN, ENABLE3_PIN);
// DRV8825 stepper4(MOTOR_STEPS, DIR4_PIN, STEP4_PIN, ENABLE4_PIN);
// DRV8825 steppers[4] = {stepper1, stepper2, stepper3, stepper4};

uint8_t serial_buff[256] = {0};
const char data[] = {1, 2, 3, 4, 5};

void setup()
{
  // pinMode(led, OUTPUT);
  // pinMode(RXPIN, INPUT);
  // pinMode(TXPIN, OUTPUT);
  Serial.setRx(RXPIN);
  Serial.setTx(TXPIN);
  Serial.begin(9600);
  // while (!digitalRead(PC6))
  // {
  //   // Serial.println("waiting");
  //   // return;
  // }
  // // uint8_t d = Serial.read();
  // Serial.println("wiufb");
  // Serial.println(d);

  // pinMode(SLEEP1_PIN, OUTPUT);
  // pinMode(RESET1_PIN, OUTPUT);

  // pinMode(SLEEP2_PIN, OUTPUT);
  // pinMode(RESET2_PIN, OUTPUT);

  // pinMode(SLEEP3_PIN, OUTPUT);
  // pinMode(RESET3_PIN, OUTPUT);

  // pinMode(SLEEP4_PIN, OUTPUT);
  // pinMode(RESET4_PIN, OUTPUT);

  // /*
  //  * Set target motor RPM.
  //  */

  // stepper1.begin(RPM);
  // stepper1.setEnableActiveState(LOW);
  // stepper1.enable();

  // stepper2.begin(RPM);
  // stepper2.setEnableActiveState(LOW);
  // stepper2.enable();

  // stepper3.begin(RPM);
  // stepper3.setEnableActiveState(LOW);
  // stepper3.enable();

  // stepper4.begin(RPM);
  // stepper4.setEnableActiveState(LOW);
  // stepper4.enable();

  // digitalWrite(SLEEP1_PIN, HIGH);
  // digitalWrite(RESET1_PIN, HIGH);

  // digitalWrite(SLEEP2_PIN, HIGH);
  // digitalWrite(RESET2_PIN, HIGH);

  // digitalWrite(SLEEP3_PIN, HIGH);
  // digitalWrite(RESET3_PIN, HIGH);

  // digitalWrite(SLEEP4_PIN, HIGH);
  // digitalWrite(RESET4_PIN, HIGH);
}
// tat net funktionieren
// int i = 0;
// void move_while_serial_reading(long steps, long time = 0L){
//     stepper1.setMicrostep(32);   // Set microstep mode to 1:8
//     stepper1.startMove(steps, time);    // forward revolution
//     long interval = stepper1.nextAction();
//     while (interval)
//     {

//       interval = stepper1.nextAction();
//       i++;
//       if (i>3){
//         i = 0;
//         Serial.println(interval);
//       }
//       if (interval > 1000){
//         continue;
//       }
//       // Serial.readBytesUntil()

//     }
// }

void clear_serial_input_buf()
{
  for (int i = 0; i < 256; i++)
  {
    serial_buff[i] = 0;
  }
}

void send_instruction(uint16_t proc_id, uint8_t instruction, uint32_t data_size, const char data[])
{
  byte proc_id_low = lowByte(proc_id);
  byte proc_id_high = highByte(proc_id);

  byte transmission[] = {START_BYTE, proc_id_high, proc_id_low, instruction};
  Serial.write(transmission, 4);
  if (data_size)
  {
    Serial.write(data, data_size);
  }
  Serial.write(END_BYTE);
}

void send_instruction(uint16_t proc_id, uint8_t instruction)
{
  byte proc_id_low = lowByte(proc_id);
  byte proc_id_high = highByte(proc_id);

  byte transmission[] = {START_BYTE, proc_id_high, proc_id_low, instruction, END_BYTE};
  Serial.write(transmission, 5);
  // transmission[3] = 130;
  // Serial.write(transmission, 5);
}

void ask_for_instructions()
{
  send_instruction(9, INSTRUCTION_PING);
  uint8_t d = -1;

  // while (true)
  // {
  // while (Serial.available() <= 0)
  // {
  //   // Serial.println("waiting");
  //   // return;
  // }
  // Serial.println("read()");
  //   if (Serial.available())
  //   {
  //     uint8_t d = Serial.read();
  //   }
  //   else
  //   {
  //     d = -1;
  //   }
  //   // Serial.println(d);
  //   if (d == -1 || d == 0xFF)
  //   {
  //     // Serial.println("returning");
  //     // return;
  //   }
  //   else if (d == START_BYTE)
  //   {
  //     break;
  //   }
  //   else
  //   {
  //     Serial.write((char)d);
  //   }
  // }
  Serial.setTimeout(10000);
  if (!Serial.find(START_BYTE))
  {
    // Serial.println("not found");
    return;
  }
  // Serial.println("found");
  // uint8_t proc_id2 = Serial.read();
  // uint8_t proc_id = Serial.read();
  // uint8_t inst_id = Serial.read();
  // Serial.print("inst_id: ");
  // Serial.println(inst_id);
  clear_serial_input_buf();
  Serial.readBytesUntil(END_BYTE, serial_buff, 256);
  uint8_t proc_id2 = serial_buff[0];
  uint8_t proc_id = serial_buff[1];
  uint8_t inst_id = serial_buff[2];
  switch (inst_id)
  {
  case INSTRUCTION_ABORT:
    // Serial.println("abort");
    // if (Serial.read() == END_BYTE)
    // {
    //   // abort();
    // }
    break;
  case INSTRUCTION_MEMORY_MAP:
    send_instruction(proc_id, INSTRUCTION_MEMORY_MAP, 5, data);
    // send_instruction(30, INSTRUCTION_MEMORY_MAP);
    break;

  default:
    Serial.print("instruction:");
    Serial.print(proc_id2, DEC);
    Serial.print(proc_id, DEC);
    Serial.println(inst_id, DEC);
    break;
  }
}

// void motor_test()
// {
//   stepper1.setMicrostep(32);
//   stepper2.setMicrostep(32);
//   stepper3.setMicrostep(32);
//   stepper4.setMicrostep(32);
//   while (true)
//   {

//     stepper1.move(32 * MOTOR_STEPS);  // reverse revolution
//     stepper1.move(-32 * MOTOR_STEPS); // reverse revolution

//     stepper2.move(32 * MOTOR_STEPS);  // forward revolution
//     stepper2.move(-32 * MOTOR_STEPS); // reverse revolution`

//     stepper3.move(32 * MOTOR_STEPS);  // forward revolution
//     stepper3.move(-32 * MOTOR_STEPS); // reverse revolution

//     stepper4.move(32 * MOTOR_STEPS);  // forward revolution
//     stepper4.move(-32 * MOTOR_STEPS); // reverse revolution
//     stepper1.rotate(360);
//   }
// }

// void serial_test()
// {
// }

// int i = 0;
void loop()
{
  ask_for_instructions();
  // digitalWrite(led, digitalRead(RXPIN));
  // if (Serial.available())
  // {
  //   Serial.print("recived: ");
  //   Serial.println(Serial.read());
  // }
  // i++;
  // if (millis() - last_cycle_time > 500) {
  //   last_cycle_time = millis();

  //   led_13 = !led_13;
  //   // pinMode(led, led_13);
  //   // Serial.println(i);
  // }
  // move_while_serial_reading(32 * MOTOR_STEPS);

  // while (true)
  // {
  //   // stepper4.move(-32 * MOTOR_STEPS);
  //   int last_loop = millis();
  //   while (millis() < last_loop + 10000)
  //   {
  //   }

  // stepper1.setMicrostep(32);
  // stepper2.setMicrostep(32);
  // stepper3.setMicrostep(32);
  // stepper4.setMicrostep(32);
  // while (true)
  // {

  //   // stepper1.move(32 * MOTOR_STEPS);  // reverse revolution
  //   // stepper1.move(-32 * MOTOR_STEPS); // reverse revolution

  //   // stepper2.move(32 * MOTOR_STEPS);  // forward revolution
  //   // stepper2.move(-32 * MOTOR_STEPS); // reverse revolution`

  //   // stepper3.move(32 * MOTOR_STEPS);  // forward revolution
  //   // stepper3.move(-32 * MOTOR_STEPS); // reverse revolution

  //   // stepper4.move(32 * MOTOR_STEPS);  // forward revolution
  //   // stepper4.move(-32 * MOTOR_STEPS); // reverse revolution
  //   delay(1000);
  //   // Serial.println("yo");
  //   ask_for_instructions();
  // }

  // ask_for_instructions();
  // while (true)
  // {
  //   Serial.println("start");
  //   ask_for_instructions();
  // }
}