#include <Arduino.h>
#include <HardwareSerial.h>
// #include <Serial.h>
#include <DRV8825.h>
#include "stepper.h"
#include <list>
#include <iostream>
using namespace std;

unsigned long last_cycle_time = 0;
bool led_13 = true;

unsigned long last_start_byte = 0;

enum Instruction
{
  wait,
  walk_stepper,
  read_sensor,
  reposition,
};

struct stepper_instruction
{
  uint8_t type;
  int16_t proc_id;

  int32_t activated_at = -1;
  uint32_t wait_time;

  uint8_t motor;
  double goal_post;
  double RPM;
};

// #define Serial Serial

#define LED_13 PC13
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

// fixed the pins
#define DIR1_PIN PC12
#define STEP1_PIN PC10
#define ENABLE1_PIN PC11
#define SLEEP1_PIN PD2
#define RESET1_PIN PB3

#define DIR2_PIN PB9
#define STEP2_PIN PB7
#define ENABLE2_PIN PB8
#define SLEEP2_PIN PC1
#define RESET2_PIN PC2

#define DIR3_PIN PA4
#define STEP3_PIN PA2
#define ENABLE3_PIN PA1
#define SLEEP3_PIN PA3
#define RESET3_PIN PA6

#define DIR4_PIN PB10
#define STEP4_PIN PA7
#define ENABLE4_PIN PC4
#define SLEEP4_PIN PB0
#define RESET4_PIN PA10

#define START_BYTE 216
#define END_BYTE 228

#define INSTRUCTION_SEND_BUFF 3
#define INSTRUCTION_FREE_MEM 4
#define INSTRUCTION_ABORT 5
#define INSTRUCTION_PAUSE 10
#define INSTRUCTION_RESUME 11
#define INSTRUCTION_RESTART 15
#define INSTRUCTION_READ 50
#define INSTRUCTION_PING 150
#define INSTRUCTION_MEMORY_MAP 130
#define INSTRUCTION_RESULT 101
#define INSTRUCTION_OK 110
#define INSTRUCTION_STATUS 102
#define INSTRUCTION_FINISHED 103
#define INSTRUCTION_ERROR 200

#define RXPIN PC7
#define TXPIN PC6

// Serial Serial(RXPIN, TXPIN);
Stepper stepper1(MOTOR_STEPS, DIR1_PIN, STEP1_PIN, ENABLE1_PIN);
Stepper stepper2(MOTOR_STEPS, DIR2_PIN, STEP2_PIN, ENABLE2_PIN);
Stepper stepper3(MOTOR_STEPS, DIR3_PIN, STEP3_PIN, ENABLE3_PIN);
Stepper stepper4(MOTOR_STEPS, DIR4_PIN, STEP4_PIN, ENABLE4_PIN);
Stepper steppers[4] = {stepper1, stepper2, stepper3, stepper4};

uint8_t serial_meta_buff[4] = {0};
uint8_t serial_data_buff[256] = {0};
char mock_data[] = {1, 2, 3, 4, 5};
#define MEMORY_SIZE 32
bool memory[MEMORY_SIZE] = {0};
string read_buffer = "";

uint32_t curr_ins = 0;
vector<stepper_instruction> step_ins;

bool paused = false;

void setup()
{
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

void clear_serial_input_buf()
{
  for (int i = 0; i < 4; i++)
  {
    serial_meta_buff[i] = 0;
  }
}

void clear_serial_input_data_buf()
{
  for (int i = 0; i < 256; i++)
  {
    serial_data_buff[i] = 0;
  }
}

void send_instruction(uint16_t proc_id, uint8_t instruction, uint8_t data_size, const char data[])
{
  uint8_t proc_id_low = lowByte(proc_id);
  uint8_t proc_id_high = highByte(proc_id);

  uint8_t transmission[] = {START_BYTE, proc_id_high, proc_id_low, instruction, data_size};
  Serial.write(transmission, 5);
  if (data_size)
  {
    Serial.write(data, data_size);
  }
  Serial.write(END_BYTE);
}

void send_instruction(uint16_t proc_id, uint8_t instruction)
{
  uint8_t proc_id_low = lowByte(proc_id);
  uint8_t proc_id_high = highByte(proc_id);

  uint8_t transmission[] = {START_BYTE, proc_id_high, proc_id_low, instruction, 0, END_BYTE};
  Serial.write(transmission, 6);
  // transmission[3] = 130;
  // Serial.write(transmission, 5);
}

void ask_for_instructions()
{
  send_instruction(9, INSTRUCTION_PING);
  uint8_t d = -1;
  bool looping = true;
  while (looping)
  {
    Serial.setTimeout(10000);
    if (!Serial.find(START_BYTE))
    {
      // Serial.println("not found");
      return;
    }
    last_start_byte = millis();
    // Serial.println("found");
    // uint8_t proc_id2 = Serial.read();
    // uint8_t proc_id = Serial.read();
    // uint8_t inst_id = Serial.read();
    // Serial.print("inst_id: ");
    // Serial.println(inst_id);
    clear_serial_input_buf();
    clear_serial_input_data_buf();
    Serial.readBytes(serial_meta_buff, 4);
    uint8_t proc_id2 = serial_meta_buff[0];
    uint8_t proc_id = serial_meta_buff[1];
    uint8_t inst_id = serial_meta_buff[2];
    uint8_t data_size = serial_meta_buff[3];

    if (data_size > 0)
    {
      Serial.readBytes(serial_data_buff, data_size);
    }

    if (Serial.read() != END_BYTE)
    {
      send_instruction(proc_id, INSTRUCTION_ERROR, 32, "endbyte not found. please repeat");
    }

    switch (inst_id)
    {
    case INSTRUCTION_SEND_BUFF:
      uint8_t a = serial_data_buff[0];
      uint8_t b = serial_data_buff[1];

      if (a >= b)
      {
        send_instruction(proc_id, INSTRUCTION_ERROR, 22, "Invalid memory range");
      }
      if (b >= MEMORY_SIZE)
      {
        send_instruction(proc_id, INSTRUCTION_ERROR, 22, "Out of memory bounds");
      }
      for (uint8_t i = a; i < b; i++)
      {
        if (memory[i])
        {
          send_instruction(proc_id, INSTRUCTION_ERROR, 22, "request overwrites existing data");
        }
        else{
          memory[i] = true;
        }
      }
      // append instructions to instruction list
      break;
    case INSTRUCTION_FREE_MEM:
      uint8_t a = serial_data_buff[0];
      uint8_t b = serial_data_buff[1];

      if (a >= b)
      {
        send_instruction(proc_id, INSTRUCTION_ERROR, 22, "Invalid memory range");
      }
      if (b >= MEMORY_SIZE)
      {
        send_instruction(proc_id, INSTRUCTION_ERROR, 22, "Out of memory bounds");
      }
      for (uint8_t i = a; i < b; i++)
      {
        memory[i] = false;
      }
      break;
    case INSTRUCTION_READ:
      uint8_t a = serial_data_buff[0];
      uint8_t b = serial_data_buff[1];

      if (a >= b)
      {
        send_instruction(proc_id, INSTRUCTION_ERROR, 22, "Invalid memory range");
      }
      if (b >= MEMORY_SIZE)
      {
        send_instruction(proc_id, INSTRUCTION_ERROR, 22, "Out of memory bounds");
      }
      for (uint8_t i = a; i < b; i++)
      {
        if (memory[i])
        {
          send_instruction(proc_id, INSTRUCTION_ERROR, 22, "cannot read from freed memory");
        }
      }
      // append instructions to instruction list
      break;
    case INSTRUCTION_ABORT:
      curr_ins = 0;
      step_ins.clear();
      send_instruction(proc_id, INSTRUCTION_OK);
      break;
    case INSTRUCTION_PAUSE:
      paused = true;
      send_instruction(proc_id, INSTRUCTION_OK);
      break;
    case INSTRUCTION_RESUME:
      paused = false;
      send_instruction(proc_id, INSTRUCTION_OK);
      break;
    case INSTRUCTION_RESTART:
      curr_ins = 0;
      send_instruction(proc_id, INSTRUCTION_OK);
      break;
    case INSTRUCTION_STATUS:
      break;
    case INSTRUCTION_ERROR:
      // wtf am i realistically supposed to do if the pc sends an error?
      // for now lets just ignore it and hope it gets handeled pc-side or the system is restated
      break;
    case INSTRUCTION_MEMORY_MAP:
      send_instruction(proc_id, INSTRUCTION_MEMORY_MAP, 5, mock_data);
      // send_instruction(30, INSTRUCTION_MEMORY_MAP);
      break;

    default:
      send_instruction(proc_id, INSTRUCTION_ERROR, 32, "uknown instruction");
      // Serial.print("instruction:");
      // Serial.print(proc_id2, DEC);
      // Serial.print(proc_id, DEC);
      // Serial.println(inst_id, DEC);
      break;
    }
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
//     stepper2.move(-32 * MOTOR_STEPS); // reverse revolution

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
  if (millis() < last_start_byte + 1000)
  {
    digitalWrite(LED_13, true);
  }

  if (step_ins.empty() || paused  || curr_ins >= step_ins.size())
  {
    return;
  }

  switch (step_ins.at(curr_ins).type)
  {

  case wait:
    if (step_ins.at(curr_ins).activated_at == -1)
    {
      step_ins.at(curr_ins).activated_at = millis();
    }
    if (millis() > step_ins.at(curr_ins).wait_time + step_ins.at(curr_ins).activated_at)
    {
      curr_ins++;
    }
    break;

  case walk_stepper:
    curr_ins++;
    break;
  case read_sensor:
    curr_ins++;
    break;
  case reposition:
    curr_ins++;
    break;
  }

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
}