#include <Arduino.h>
#include <HardwareSerial.h>
// #include <Serial.h>
#include <DRV8825.h>
#include "stepper.h"
#include <list>
#include <iostream>
#include <ezButton.h>
using namespace std;

// #define Serial Serial

#define LED_13 PC13
// HardwareSerial Serial(USART6);
// from datasheet
#define MOTOR_STEPS 200
#define RPM_VERTICAL 300
#define RPM_HORIZONTAL 50

#define WORKING_HEIGHT 0
#define CANVAS_HEIGHT 0
#define CANVAS_X 0
#define cANVAS_Y 0
#define CANVAS_H 0
#define cANVAS_W 0
// #define

// fixed the pins
#define EDGE1_PIN PA15
#define DIR1_PIN PC12
#define STEP1_PIN PC10
#define ENABLE1_PIN PC11
#define SLEEP1_PIN PD2
#define RESET1_PIN PB3

#define EDGE2_PIN PB5
#define DIR2_PIN PB9
#define STEP2_PIN PB7
#define ENABLE2_PIN PB8
#define SLEEP2_PIN PC1
#define RESET2_PIN PC2

#define EDGE3_PIN PC3
#define DIR3_PIN PA4
#define STEP3_PIN PA2
#define ENABLE3_PIN PA1
#define SLEEP3_PIN PA3
#define RESET3_PIN PA6

#define EDGE4_PIN PB1
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

#define CANVAS_COLUMNS 20
#define CANVAS_ROWS 12
#define CANVAS_DOTS_COUNT CANVAS_COLUMNS *CANVAS_ROWS
#define MEMORY_SIZE CANVAS_DOTS_COUNT / 8

#define RXPIN PC7
#define TXPIN PC6

bool motor_dir = true;

unsigned long last_cycle_time = 0;
bool led_13 = true;

unsigned long last_start_byte = 0;

enum Instruction
{
    wait,
    walk_stepper,
    read_sensor,
    edge,
};

enum Status_type
{
    no_operation,
    status_working,
    status_paused,
    status_standby,
};

struct stepper_instruction
{
    uint8_t type;
    int16_t proc_id;

    int32_t activated_at = -1;
    uint32_t wait_time;

    uint8_t motor;
    double goal_post;
    double rpm;
};

// Serial Serial(RXPIN, TXPIN);
Stepper stepper1(MOTOR_STEPS, DIR1_PIN, STEP1_PIN, SLEEP1_PIN);
Stepper stepper2(MOTOR_STEPS, DIR2_PIN, STEP2_PIN, SLEEP2_PIN);
Stepper stepper3(MOTOR_STEPS, DIR3_PIN, STEP3_PIN, SLEEP3_PIN);
Stepper stepper4(MOTOR_STEPS, DIR4_PIN, STEP4_PIN, SLEEP4_PIN);
Stepper steppers[4] = {stepper1, stepper2, stepper3, stepper4};

uint8_t a;
uint8_t b;
uint8_t serial_meta_buff[4] = {0};
uint8_t serial_data_buff[256] = {0};
char mock_data[] = {1, 2, 3, 4, 5};
#define MEMORY_SIZE 32
bool memory[MEMORY_SIZE] = {1};
string read_buffer = "";

uint32_t curr_ins = 0;
vector<stepper_instruction> step_ins;

vector<vector<pair<int, int>>> presets;

bool paused = false;

ezButton button(PA0); // create ezButton object that attach to pin 7

// pinMode(PC13, OUTPUT);

std::vector<std::pair<int, int>> generate_preset1()
{
    std::vector<std::pair<int, int>> preset;
    for (int y = 0; y < CANVAS_ROWS - 1; y += 2)
    {
        for (int x = 0; x < CANVAS_COLUMNS; ++x)
        {
            if ((y / 2) % 2)
            {
                if (x % 2)
                {
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y));
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y + 1));
                }
                else
                {
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y + 1));
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y));
                }
            }
            else
            {
                if (x % 2)
                {
                    preset.push_back(std::make_pair(x, y));
                    preset.push_back(std::make_pair(x, y + 1));
                }
                else
                {
                    preset.push_back(std::make_pair(x, y + 1));
                    preset.push_back(std::make_pair(x, y));
                }
            }
        }
    }
    if (CANVAS_ROWS % 2)
    {
        if ((CANVAS_ROWS / 2) % 2)
        {
            for (int x = 0; x < CANVAS_COLUMNS; ++x)
            {
                preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, CANVAS_ROWS - 1));
            }
        }
        else
        {
            for (int x = 0; x < CANVAS_COLUMNS; ++x)
            {
                preset.push_back(std::make_pair(x, CANVAS_ROWS - 1));
            }
        }
    }
    return preset;
}

std::vector<std::pair<int, int>> generate_preset2()
{
    std::vector<std::pair<int, int>> preset;
    for (int x = 0; x < CANVAS_COLUMNS; ++x)
    {
        preset.push_back(std::make_pair(x, 0));
    }

    for (int y = 1; y < CANVAS_ROWS - 1; y += 2)
    {
        for (int x = 0; x < CANVAS_COLUMNS; ++x)
        {
            if (!((y / 2) % 2))
            {
                if (x % 2)
                {
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y));
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y + 1));
                }
                else
                {
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y + 1));
                    preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, y));
                }
            }
            else
            {
                if (x % 2)
                {
                    preset.push_back(std::make_pair(x, y));
                    preset.push_back(std::make_pair(x, y + 1));
                }
                else
                {
                    preset.push_back(std::make_pair(x, y + 1));
                    preset.push_back(std::make_pair(x, y));
                }
            }
        }
    }

    if (!(CANVAS_ROWS % 2))
    {
        if ((CANVAS_ROWS / 2) % 2)
        {
            for (int x = 0; x < CANVAS_COLUMNS; ++x)
            {
                preset.push_back(std::make_pair(CANVAS_COLUMNS - 1 - x, CANVAS_ROWS - 1));
            }
        }
        else
        {
            for (int x = 0; x < CANVAS_COLUMNS; ++x)
            {
                preset.push_back(std::make_pair(x, CANVAS_ROWS - 1));
            }
        }
    }
    return preset;
}

long last_motor_1_edging = -1000;
long last_motor_2_edging = -1000;
long last_motor_3_edging = -1000;

bool is_motor1_edging()
{
    if (!digitalRead(EDGE1_PIN))
    {
        last_motor_1_edging = millis();
    }
    if (last_motor_1_edging + 1000 > millis())
    {
        return true;
    }
    return false;
}

bool is_motor2_edging()
{
    if (analogRead(EDGE4_PIN) < 400)
    {
        last_motor_2_edging = millis();
    }
    if (last_motor_2_edging + 1000 > millis())
    {
        return true;
    }
    return false;
}

bool is_motor3_edging()
{
    if (analogRead(EDGE3_PIN) < 900)
    {
        last_motor_3_edging = millis();
    }
    if (last_motor_3_edging + 1000 > millis())
    {
        return true;
    }
    return false;
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
    while (millis() < last_start_byte + 150)
    {
    }
    uint8_t proc_id_low = lowByte(proc_id);
    uint8_t proc_id_high = highByte(proc_id);

    uint8_t transmission[] = {START_BYTE, proc_id_high, proc_id_low, instruction, data_size, END_BYTE};
    Serial.write(transmission, 6);
    if (data_size)
    {
        Serial.write(data, data_size);
    }
    Serial.write(END_BYTE);
}

void send_instruction(uint16_t proc_id, uint8_t instruction)
{
    while (millis() < last_start_byte + 150)
    {
    }
    uint8_t proc_id_low = lowByte(proc_id);
    uint8_t proc_id_high = highByte(proc_id);

    uint8_t transmission[] = {START_BYTE, proc_id_high, proc_id_low, instruction, 0, END_BYTE};
    Serial.write(transmission, 6);
    // transmission[3] = 130;
    // Serial.write(transmission, 5);
}

void ask_for_instructions(uint32_t timeout = 10000)
{
    send_instruction(9, INSTRUCTION_PING);
    uint8_t d = -1;
    bool looping = true;
    while (looping)
    {
        Serial.setTimeout(timeout);
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
        Serial.readBytes(serial_meta_buff, 5);
        // uint8_t f = Serial.read();
        uint8_t proc_id2 = serial_meta_buff[0];
        uint8_t proc_id = serial_meta_buff[1];
        uint8_t inst_id = serial_meta_buff[2];
        uint8_t data_size = serial_meta_buff[3];
        uint8_t end_byte = serial_meta_buff[4];
        Serial.print("instruction: "); // Add a space for better readability
        Serial.print(inst_id, DEC);
        Serial.println(); // Print a newline character to start a new line
        if (end_byte != END_BYTE)
        {
            send_instruction(proc_id, INSTRUCTION_ERROR, 32, "endbyte not found. please repeat");
            // for (int i = 0; i < sizeof(serial_meta_buff); i++)
            // {
            //   Serial.print(serial_meta_buff[i]);
            //   Serial.print(" "); // Add a space for better readability
            // }
            // for (int i = 0; i < sizeof(serial_data_buff); i++)
            // {
            //   Serial.print(serial_data_buff[i], HEX);
            //   Serial.print(" "); // Add a space for better readability
            // }
            // Serial.print("end)_byte: "); // Add a space for better readability
            // Serial.print(end_byte, DEC);
            // Serial.println(); // Print a newline character to start a new line
        }

        if (data_size > 0)
        {
            Serial.readBytes(serial_data_buff, data_size + 1);
            if (serial_data_buff[data_size] != END_BYTE)
            {
                send_instruction(proc_id, INSTRUCTION_ERROR, 32, "endbyte not found. please repeat");
            }
        }

        switch (inst_id)
        {
        case INSTRUCTION_SEND_BUFF:
            a = serial_data_buff[0];
            b = serial_data_buff[1];

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
                else
                {
                    memory[i] = true;
                }
            }
            send_instruction(proc_id, INSTRUCTION_OK);

            for (int j = 0; j < 15; j++)
            {
                // std::vector<stepper_instruction> step_ins
                stepper_instruction inst;
                inst.type = wait;
                inst.wait_time = 1000;
                step_ins.push_back(inst);
            }

            // append instructions to instruction list
            break;
        case INSTRUCTION_FREE_MEM:
            a = serial_data_buff[0];
            b = serial_data_buff[1];

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
            send_instruction(proc_id, INSTRUCTION_OK);
            break;
        case INSTRUCTION_READ:
            a = serial_data_buff[0];
            b = serial_data_buff[1];

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
            send_instruction(proc_id, INSTRUCTION_READ, read_buffer.size(), read_buffer.c_str());
            // append instructions to instruction list
            break;
        case INSTRUCTION_ABORT:
            curr_ins = 0;
            step_ins.clear();
            send_instruction(proc_id, INSTRUCTION_OK);
            break;
        case INSTRUCTION_PAUSE:
            paused = true;
            Serial.println("got here");
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
            char temp_data[5];
            if (paused)
            {
                temp_data[0] = (uint8_t)status_paused;
            }
            else if (step_ins.empty())
            {
                temp_data[0] = (uint8_t)status_standby;
            }
            else
            {
                temp_data[0] = (uint8_t)status_working;
            }
            temp_data[1] = (uint8_t)highByte(curr_ins);
            temp_data[2] = (uint8_t)lowByte(curr_ins);
            temp_data[3] = (uint8_t)highByte(step_ins.size());
            temp_data[4] = (uint8_t)lowByte(step_ins.size());
            send_instruction(proc_id, INSTRUCTION_STATUS, 5, temp_data);
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

// int start_time = -10000;
void loop()
{
    // stepper2.enable();
    // for (int i = 0; i < 15; i++)
    // {
    //     Serial.print(">alanog4: ");
    //     Serial.println(analogRead(EDGE4_PIN));
    //     Serial.print(">cstm4: ");
    //     Serial.println(is_motor2_edging());
    //     Serial.print(">alanog3: ");
    //     Serial.println(analogRead(EDGE3_PIN));
    //     Serial.print(">cstm3: ");
    //     Serial.println(is_motor3_edging());
    //     stepper2.walk(5);
    // }
    // stepper2.disable();

    // stepper3.enable();
    // for (int i = 0; i < 15; i++)
    // {
    //     Serial.print(">alanog4: ");
    //     Serial.println(analogRead(EDGE4_PIN));
    //     Serial.print(">cstm4: ");
    //     Serial.println(is_motor2_edging());
    //     Serial.print(">alanog3: ");
    //     Serial.println(analogRead(EDGE3_PIN));
    //     Serial.print(">cstm3: ");
    //     Serial.println(is_motor3_edging());
    //     stepper3.walk(5);
    // }
    // stepper3.disable();

    // stepper2.enable();
    // for (int i = 0; i < 15; i++)
    // {
    //     Serial.print(">alanog4: ");
    //     Serial.println(analogRead(EDGE4_PIN));
    //     Serial.print(">cstm4: ");
    //     Serial.println(is_motor2_edging());
    //     Serial.print(">alanog3: ");
    //     Serial.println(analogRead(EDGE3_PIN));
    //     Serial.print(">cstm3: ");
    //     Serial.println(is_motor3_edging());
    //     stepper2.walk(-5);
    // }
    // stepper2.disable();

    // stepper3.enable();
    // for (int i = 0; i < 15; i++)
    // {
    //     Serial.print(">alanog4: ");
    //     Serial.println(analogRead(EDGE4_PIN));
    //     Serial.print(">cstm4: ");
    //     Serial.println(is_motor2_edging());
    //     Serial.print(">alanog3: ");
    //     Serial.println(analogRead(EDGE3_PIN));
    //     Serial.print(">cstm3: ");
    //     Serial.println(is_motor3_edging());
    //     stepper3.walk(-5);
    // }
    // stepper3.disable();

    // while (!is_motor3_edging())
    // {
    //   Serial.print(">cstm4: ");
    //   Serial.println(is_motor2_edging());
    //   stepper2.walk(1);
    // }
    // while ()
    //   Serial.print(">cstm4: ");
    // Serial.println(is_motor2_edging());
    // delay(500);
    // stepper2.enable();
    // stepper2.walk(-220); // reverse revolution
    // stepper2.disable();

    //   button.loop();
    //   Serial.print(">alanog: ");
    //   Serial.println(analogRead(EDGE1_PIN));
    //   Serial.print(">fifital: ");
    //   Serial.println(digitalRead(EDGE1_PIN));
    //   // Serial.print(">v2: ");
    //   // Serial.println(analogRead(EDGE1_PIN));
    //   // Serial.print(">v3: ");
    //   // Serial.println(analogRead(EDGE2_PIN));
    //   // Serial.print(">v4: ");
    //   // Serial.println(analogRead(EDGE3_PIN));
    //   if (!digitalRead(EDGE1_PIN) || button.isReleased())
    //   {
    //     Serial.println("pressed1");
    //   }
    //   // if (!digitalRead(EDGE2_PIN) || button.isReleased())
    //   // {
    //   //   Serial.println("pressed2");
    //   // }

    //   // if (!digitalRead(EDGE3_PIN))
    //   // {
    //   //   Serial.println("pressed3");
    // }
    // if (!digitalRead(EDGE4_PIN))
    // {
    //   Serial.println("pressed4");
    // }
    // if (!digitalRead(EDGE3_PIN))
    // {
    //   Serial.print("pressed3");
    // }
    // button.loop();
    // stepper1.setMicrostep(32);
    // // stepper2.setMicrostep(32);
    // // stepper3.setMicrostep(32);
    // // stepper4.setMicrostep(32);
    // // while (true)
    // // {

    // delay(500);
    // stepper3.enable();
    // stepper3.walk(50); // reverse revolution
    // stepper3.disable();
    // delay(500);
    // stepper3.enable();
    // stepper3.walk(-50); // reverse revolution
    // stepper3.disable();

    // int btnState = button.getState();
    // if (button.isReleased())
    // {
    //   digitalWrite(SLEEP1_PIN, HIGH);
    //   motor_dir = !motor_dir;
    //   Serial.println("rotating");
    //   if (motor_dir)
    //   {
    //     for (int j = 0; j < 50; j++)
    //     {
    //       if (!digitalRead(EDGE1_PIN))
    //       {
    //         break;
    //       }
    //       stepper1.walk(1);
    //     }
    //   }
    //   else
    //   {
    //     for (int j = 0; j < 50; j++)
    //     {
    //       if (!digitalRead(EDGE1_PIN))
    //       {
    //         break;
    //       }
    //       stepper1.walk(-1);
    //     }
    //   }
    //   Serial.println("finished rotating");
    //   digitalWrite(SLEEP1_PIN, LOW);
    //   // stepper1.rotate(360);
    // }

    /// actual code!!!
    ask_for_instructions(3000);
    if (millis() < last_start_byte + 1000)
    {
        digitalWrite(LED_13, true);
    }

    if (step_ins.empty() || paused || curr_ins >= step_ins.size())
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
        digitalWrite(SLEEP1_PIN, HIGH);
        steppers[step_ins.at(curr_ins).motor].walk(50);
        digitalWrite(SLEEP1_PIN, LOW);
        curr_ins++;
        break;

    case read_sensor:

        curr_ins++;
        break;

    case edge:

        stepper3.enable();
        while (!is_motor3_edging())
        {
            stepper3.walk(1);
        }
        stepper3.disable();
        delay(500);
        stepper3.enable();
        stepper3.walk(-220); // reverse revolution
        stepper3.disable();

        delay(500);

        stepper2.enable();
        while (!is_motor2_edging())
        {
            stepper2.walk(1);
        }
        stepper2.disable();
        delay(500);
        stepper2.enable();
        stepper2.walk(-195); // reverse revolution
        stepper2.disable();

        delay(500);

        stepper1.enable();
        while (!is_motor1_edging())
        {
            stepper1.walk(1);
        }
        stepper1.disable();
        delay(500);
        stepper1.enable();
        stepper1.walk(-5); // reverse revolution
        stepper1.disable();
        curr_ins++;
        break;
    }

    // end of actual code

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

void setup()
{
    pinMode(EDGE1_PIN, INPUT);
    pinMode(EDGE2_PIN, INPUT);
    pinMode(EDGE3_PIN, INPUT);
    pinMode(EDGE4_PIN, INPUT);
    digitalWrite(EDGE1_PIN, LOW);
    digitalWrite(EDGE2_PIN, LOW);
    digitalWrite(EDGE3_PIN, LOW);
    digitalWrite(EDGE4_PIN, LOW);

    // set the EDGE Pins pull down
    // (n * 2) indicates the specific pin of the port
    // since every pin has two bits we multiply the index by two
    GPIOA->PUPDR << 0b10 << (15 * 2);
    GPIOB->PUPDR << 0b10 << (5 * 2);
    GPIOC->PUPDR << 0b10 << (3 * 2);
    GPIOB->PUPDR << 0b10 << (1 * 2);

    presets.push_back(generate_preset1());
    presets.push_back(generate_preset2());
    Serial.setRx(RXPIN);
    Serial.setTx(TXPIN);
    Serial.begin(9600);

    // Serial.print("[");
    // for (const auto &preset : presets)
    // {
    //   Serial.print("[");
    //   for (const auto &pair : preset)
    //   {
    //     Serial.print("[");
    //     Serial.print(pair.first);
    //     Serial.print(",");
    //     Serial.print(pair.second);
    //     Serial.println("] ");
    //   }
    //   Serial.println("] ");
    // }
    // Serial.println("] ");

    // while (!digitalRead(PC6))
    // {
    //   // Serial.println("waiting");
    //   // return;
    // }
    // // uint8_t d = Serial.read();
    // Serial.println("wiufb");
    // Serial.println(d);

    pinMode(ENABLE1_PIN, OUTPUT);
    pinMode(RESET1_PIN, OUTPUT);

    pinMode(ENABLE2_PIN, OUTPUT);
    pinMode(RESET2_PIN, OUTPUT);

    pinMode(ENABLE3_PIN, OUTPUT);
    pinMode(RESET3_PIN, OUTPUT);

    pinMode(ENABLE4_PIN, OUTPUT);
    pinMode(RESET4_PIN, OUTPUT);

    // /*
    //  * Set target motor RPM.
    //  */

    stepper1.begin(RPM_VERTICAL, 32);
    stepper1.setEnableActiveState(HIGH);
    stepper1.disable();
    stepper1.degPermm = 0.003333333;

    stepper2.begin(RPM_HORIZONTAL, 32);
    stepper2.setEnableActiveState(HIGH);
    stepper2.disable();
    stepper2.degPermm = 0.138888889;

    stepper3.begin(RPM_HORIZONTAL, 32);
    stepper3.setEnableActiveState(HIGH);
    stepper3.disable();
    stepper3.degPermm = 0.138888889;

    stepper4.begin(RPM_HORIZONTAL, 32);
    stepper4.setEnableActiveState(HIGH);
    stepper4.disable();
    stepper4.degPermm = 0.138888889;

    digitalWrite(ENABLE1_PIN, LOW);
    digitalWrite(RESET1_PIN, HIGH);

    digitalWrite(ENABLE2_PIN, LOW);
    digitalWrite(RESET2_PIN, HIGH);

    digitalWrite(ENABLE3_PIN, LOW);
    digitalWrite(RESET3_PIN, HIGH);

    digitalWrite(ENABLE4_PIN, LOW);
    digitalWrite(RESET4_PIN, HIGH);

    // stepper3.enable();
    // while (!is_motor3_edging())
    // {
    //   Serial.print(">cstm4: ");
    //   Serial.println(is_motor2_edging());
    //   Serial.print(">cstm3: ");
    //   Serial.println(is_motor3_edging());
    //   stepper3.walk(1);
    // }
    // Serial.print(">cstm4: ");
    // Serial.println(is_motor2_edging());
    // Serial.print(">cstm3: ");
    // Serial.println(is_motor3_edging());
    // stepper3.disable();
    // delay(500);
    // stepper3.enable();
    // stepper3.walk(-220); // reverse revolution
    // stepper3.disable();

    // delay(500);

    // stepper2.enable();
    // while (!is_motor2_edging())
    // {
    //   stepper2.walk(1);
    //   Serial.print(">cstm4: ");
    //   Serial.println(is_motor2_edging());
    //   Serial.print(">cstm3: ");
    //   Serial.println(is_motor3_edging());
    // }
    // Serial.print(">cstm4: ");
    // Serial.println(is_motor2_edging());
    // Serial.print(">cstm3: ");
    // Serial.println(is_motor3_edging());
    // stepper2.disable();
    // delay(500);
    // stepper2.enable();
    // stepper2.walk(-195); // reverse revolution
    // stepper2.disable();

    // delay(500);

    //   stepper1.enable();
    //   while (!is_motor1_edging())
    //   {
    //     stepper1.walk(1);
    //   }
    //   stepper1.disable();
    //   delay(500);
    //   stepper1.enable();
    //   stepper1.walk(-200); // reverse revolution
    //   stepper1.disable();
}