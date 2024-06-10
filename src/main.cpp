#include <Arduino.h>
#include <HardwareSerial.h>
// #include <Serial.h>
#include <DRV8825.h>
#include "stepper.h"
#include <list>
#include <iostream>
#include <ezButton.h>
#include "antcolony.h"
#include <ezButton.h>
using namespace std;

// #define Serial Serial

#define LED_13 PC13
#define LDR_PIN PB6
// HardwareSerial Serial(USART6);
// from datasheet
#define MOTOR_STEPS 200
#define RPM_VERTICAL 300
#define RPM_HORIZONTAL 25

// cnavas ppos constants
#define WRITING_CRUISING_HEIGHT 9
#define READING_CRUISING_HEIGHT 4
#define READ_HEIGHT 0
#define WRITE_HEIGHT 1

#define CANVAS_WRITE_X 105
#define CANVAS_WRITE_Y 30

#define CANVAS_READ_X CANVAS_WRITE_X - 103
#define CANVAS_READ_Y CANVAS_WRITE_Y - 5

#define CANVAS_H 140
#define cANVAS_W 100

#define DOT2DOT_DISTANCE 10
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
#define INSTRUCTION_WALK 90
#define INSTRUCTION_EDGE 91
#define INSTRUCTION_READ 50
#define INSTRUCTION_PING 150
#define INSTRUCTION_MEMORY_MAP 130
#define INSTRUCTION_RESULT 101
#define INSTRUCTION_OK 110
#define INSTRUCTION_STATUS 102
#define INSTRUCTION_FINISHED 103
#define INSTRUCTION_ERROR 200

#define CANVAS_COLUMNS 10
#define CANVAS_ROWS 15
#define CANVAS_DOTS_COUNT CANVAS_COLUMNS *CANVAS_ROWS
#define MEMORY_SIZE CANVAS_DOTS_COUNT / 8

#define RXPIN PC7
#define TXPIN PC6

ezButton edger(EDGE1_PIN);

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

    uint16_t read_pos;

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
bool memory[MEMORY_SIZE] = {0};
char read_buffer[256] = "";
uint8_t read_buffer_size = 0;

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

int motor_1_edger_pressed_for = 0;

bool is_motor1_edging()
{
    // if (!(millis() % 20))
    // {
    //     Serial.println(!digitalRead(EDGE1_PIN));
    // }
    if (!digitalRead(EDGE1_PIN))
    {
        // Serial.println(!digitalRead(EDGE1_PIN));
        // Serial.println(edger.getState());
        last_motor_1_edging = millis();
        motor_1_edger_pressed_for++;
    }
    else
    {
        motor_1_edger_pressed_for = 0;
    }
    if (last_motor_1_edging + 1000 > millis() &&
        motor_1_edger_pressed_for > 4)
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
    if (analogRead(EDGE3_PIN) < 400)
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

void clear_read_buf()
{
    for (int i = 0; i < 256; i++)
    {
        read_buffer[i] = ' ';
    }
}

void send_instruction(uint16_t proc_id, uint8_t instruction, uint8_t data_size, char data[])
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

void pathfinder_write(uint16_t a, uint16_t b)
{
    vector<vector<pair<int, int>>> paths;
    int curr_distance = 9999999;
    vector<pair<int, int>> longus_path;
    vector<pair<double, double>> to_path;

    vector<int> filtered_preset;
    vector<pair<int, int>> path;

    for (int i = a; i < b; i++)
    {
        if (i % 8 == 0)
        {
            Serial.println("");
            Serial.print("byte nr ");
            Serial.print(i / 8);
            Serial.print(": ");
        }

        if (bitRead(serial_data_buff[(i / 8) + 2], i % 8))
        {
            Serial.print(1);
            to_path.push_back(pair((double)(i % CANVAS_COLUMNS), (double)(i / CANVAS_COLUMNS)));
        }
        else
        {
            Serial.print(0);
        }
    }

    // for (const auto &preset : presets)
    // {
    //     for (const auto &p1 : to_path)
    //     {
    //         for (int i = 0; i < preset.size(); i++)
    //         {
    //             if (preset[i].first == p1.first && preset[i].second == p1.second)
    //             {
    //                 filtered_preset.push_back(i);
    //                 break;
    //             }
    //         }

    //         // for i in filtered_preset:
    //         //     if preset[i] in to_path and preset[i]:
    //         //         p.append(preset[i])
    //     }
    //     sort(filtered_preset.begin(), filtered_preset.end());
    //     for (const auto &i : filtered_preset)
    //     {
    //         if (std::find(to_path.begin(), to_path.end(), preset.at(i)) != to_path.end())
    //         {
    //             path.push_back(preset.at(i));
    //         }
    //     }
    //     uint16_t straight_distance = 0;
    //     for (int i = 1; i < path.size(); i++)
    //     {
    //         if (path[i].second == path[i - 1].second)
    //         {
    //             straight_distance++;
    //         }
    //     }
    //     if (straight_distance < curr_distance)
    //     {
    //         curr_distance = straight_distance;
    //         longus_path = path;
    //     }
    // Serial.print(a);
    // Serial.print(",");
    // // Serial.print(b);
    // Serial.print("[");
    // for (const auto &pair : to_path)
    // {
    //     Serial.print("[");
    //     Serial.print(pair.first);
    //     Serial.print(",");
    //     Serial.print(pair.second);
    //     Serial.print("], ");
    // }
    // Serial.println("] ");
    // }

    AntColony aco(to_path, 10, 20, 0.5, 1, 2, 0.1);
    aco.run();
    std::vector<std::pair<double, double>> ant_path;
    // std::vector<std::pair<double, double>> ant_path;
    longus_path.clear();
    // for (int i = 0; i < aco.global_best_path.size(); i++)
    // {
    //     // std::pair<double, double> p = {0, 0};
    //     // longus_path[i].first = (int)to_path[i].first;
    //     // longus_path[i].second = (int)to_path[i].second;
    //     longus_path.push_back(pair((int)to_path[i].first, (int)to_path[i].second));
    // }
    longus_path = aco.get_best_path();

    // if (aco.global_best_distance < curr_distance)
    // {
    //     curr_distance = aco.global_best_distance;
    //     longus_path = ant_path;
    // }

    Serial.print("[");
    for (const auto &pair : longus_path)
    {
        Serial.print("[");
        Serial.print(pair.first);
        Serial.print(",");
        Serial.print(pair.second);
        Serial.print("], ");
    }
    Serial.println("] ");

    stepper_instruction highherad;
    highherad.type = walk_stepper;
    highherad.motor = 1;
    highherad.goal_post = WRITING_CRUISING_HEIGHT;
    step_ins.push_back(highherad);

    for (const auto &pair : longus_path)
    {
        stepper_instruction inst1;
        inst1.type = walk_stepper;
        inst1.motor = 3;
        inst1.goal_post = CANVAS_WRITE_X + (pair.first * DOT2DOT_DISTANCE);
        step_ins.push_back(inst1);

        stepper_instruction inst2;
        inst2.type = walk_stepper;
        inst2.motor = 2;
        inst2.goal_post = CANVAS_WRITE_Y + (pair.second * DOT2DOT_DISTANCE);
        step_ins.push_back(inst2);

        stepper_instruction inst3;
        inst3.type = walk_stepper;
        inst3.motor = 1;
        inst3.goal_post = WRITE_HEIGHT;
        step_ins.push_back(inst3);

        stepper_instruction inst4;
        inst4.type = walk_stepper;
        inst4.motor = 1;
        inst4.goal_post = WRITING_CRUISING_HEIGHT;
        step_ins.push_back(inst4);
    }

    // Serial.print("[");
    // for (const auto &preset : presets)
    // {
    // }
    // Serial.println("] ");
}

void pathfinder_read(uint8_t a, uint8_t b)
{
    vector<vector<pair<int, int>>> paths;
    int curr_distance = 9999999;
    vector<pair<int, int>> longus_path;

    for (const auto &preset : presets)
    {
        vector<pair<int, int>> to_path;
        vector<int> filtered_preset;
        vector<pair<int, int>> path;

        for (int i = a; i < b; i++)
        {
            to_path.push_back(pair(i % CANVAS_COLUMNS, i / CANVAS_COLUMNS));
        }

        for (const auto &p1 : to_path)
        {
            for (int i = 0; i < preset.size(); i++)
            {
                if (preset[i].first == p1.first && preset[i].second == p1.second)
                {
                    filtered_preset.push_back(i);
                    break;
                }
            }

            // for i in filtered_preset:
            //     if preset[i] in to_path and preset[i]:
            //         p.append(preset[i])
        }
        sort(filtered_preset.begin(), filtered_preset.end());
        for (const auto &i : filtered_preset)
        {
            if (std::find(to_path.begin(), to_path.end(), preset.at(i)) != to_path.end())
            {
                path.push_back(preset.at(i));
            }
        }
        uint16_t straight_distance = 0;
        for (int i = 1; i < path.size(); i++)
        {
            if (path[i].second == path[i - 1].second)
            {
                straight_distance++;
            }
        }
        if (straight_distance < curr_distance)
        {
            curr_distance = straight_distance;
            longus_path = path;
        }
        // Serial.print(a);
        // Serial.print(",");
        // // Serial.print(b);
        // Serial.print("[");
        // for (const auto &pair : to_path)
        // {
        //     Serial.print("[");
        //     Serial.print(pair.first);
        //     Serial.print(",");
        //     Serial.print(pair.second);
        //     Serial.print("], ");
        // }
        // Serial.println("] ");
    }
    Serial.print("[");
    for (const auto &pair : longus_path)
    {
        Serial.print("[");
        Serial.print(pair.first);
        Serial.print(",");
        Serial.print(pair.second);
        Serial.print("], ");
    }
    Serial.println("] ");

    stepper_instruction highherad;
    highherad.type = walk_stepper;
    highherad.motor = 1;
    highherad.goal_post = WRITING_CRUISING_HEIGHT;
    step_ins.push_back(highherad);
    int i = 0;
    for (const auto &pair : longus_path)
    {
        stepper_instruction inst1;
        inst1.type = walk_stepper;
        inst1.motor = 3;
        inst1.goal_post = CANVAS_READ_X + (pair.first * DOT2DOT_DISTANCE);
        step_ins.push_back(inst1);

        stepper_instruction inst2;
        inst2.type = walk_stepper;
        inst2.motor = 2;
        inst2.goal_post = CANVAS_READ_Y + (pair.second * DOT2DOT_DISTANCE);
        step_ins.push_back(inst2);

        // if (i == 0)
        // {
        //     stepper_instruction lower_head;
        //     lower_head.type = walk_stepper;
        //     lower_head.motor = 1;
        //     lower_head.goal_post = READ_HEIGHT;
        //     step_ins.push_back(lower_head);
        // }
        stepper_instruction lower_head;
        lower_head.type = walk_stepper;
        lower_head.motor = 1;
        lower_head.goal_post = READ_HEIGHT;
        step_ins.push_back(lower_head);

        stepper_instruction inst3;
        inst3.type = read_sensor;
        inst3.read_pos = ((CANVAS_COLUMNS * pair.second) + pair.first) - a;
        step_ins.push_back(inst3);

        stepper_instruction elevate_head;
        elevate_head.type = walk_stepper;
        elevate_head.motor = 1;
        elevate_head.goal_post = READING_CRUISING_HEIGHT;
        step_ins.push_back(elevate_head);

        i++;
    }

    stepper_instruction elevate_head;
    elevate_head.type = walk_stepper;
    elevate_head.motor = 1;
    elevate_head.goal_post = READING_CRUISING_HEIGHT;
    step_ins.push_back(elevate_head);

    // Serial.print("[");
    // for (const auto &preset : presets)
    // {
    // }
    // Serial.println("] ");
}

void pathfinder(vector<pair<int, int>> path, bool write = true)
{
    Serial.print("[");
    for (const auto &pair : path)
    {
        Serial.print("[");
        Serial.print(pair.first);
        Serial.print(",");
        Serial.print(pair.second);
        Serial.print("], ");
    }
    Serial.println("] ");
    int x = CANVAS_READ_X;
    int y = CANVAS_READ_Y;
    if (write)
    {
        x = CANVAS_WRITE_X;
        y = CANVAS_WRITE_Y;
    }

    for (const auto &pair : path)
    {
        if (pair.first == -1 || pair.second == -1)
        {
            stepper_instruction inst1;
            inst1.type = walk_stepper;
            inst1.motor = 1;
            inst1.goal_post = READ_HEIGHT;
            step_ins.push_back(inst1);
        }
        else if (pair.first == -2 || pair.second == -2)
        {
            stepper_instruction inst1;
            inst1.type = walk_stepper;
            inst1.motor = 1;
            inst1.goal_post = WRITING_CRUISING_HEIGHT;
            step_ins.push_back(inst1);
        }
        else
        {
            stepper_instruction inst1;
            inst1.type = walk_stepper;
            inst1.motor = 3;
            inst1.goal_post = x + (pair.first * DOT2DOT_DISTANCE);
            step_ins.push_back(inst1);

            stepper_instruction inst2;
            inst2.type = walk_stepper;
            inst2.motor = 2;
            inst2.goal_post = y + (pair.second * DOT2DOT_DISTANCE);
            step_ins.push_back(inst2);
        }
    }
}

void ask_for_instructions(uint32_t timeout = 500)
{
    send_instruction(9, INSTRUCTION_PING);
    uint8_t d = -1;
    bool looping = true;
    while (looping)
    {
        Serial.setTimeout(timeout);
        if (!Serial.find(START_BYTE))
        {
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
        Serial.print("inst_id: ");
        Serial.print(inst_id);
        // Serial.print(",");
        // Serial.print(pair.second);
        // Serial.print("], ");
        if (inst_id == INSTRUCTION_SEND_BUFF)
        {
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
            if (b - a > data_size)
            {
                send_instruction(proc_id, INSTRUCTION_ERROR, 34, "data size is smalle than a minus b");
            }
            bool mem_fault = false;
            for (uint8_t i = a; i < b; i++)
            {
                if (memory[i])
                {
                    bool mem_fault = false;
                }
                else
                {
                    memory[i] = true;
                }
            }
            if (mem_fault)
            {
                send_instruction(proc_id, INSTRUCTION_ERROR, 32, "request overwrites existing data");
            }
            send_instruction(proc_id, INSTRUCTION_OK);
            pathfinder_write(a * 8, b * 8);

            // for (int j = 0; j < 15; j++)
            // {
            //     // std::vector<stepper_instruction> step_ins
            //     stepper_instruction inst;
            //     inst.type = wait;
            //     inst.wait_time = 1000;
            //     step_ins.push_back(inst);
            // }

            // append instructions to instruction list}
        }
        else if (inst_id == INSTRUCTION_FREE_MEM)
        {
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
        }
        else if (inst_id == INSTRUCTION_READ)
        {
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
            bool mem_fault = false;
            for (uint8_t i = a; i < b; i++)
            {
                if (!memory[i])
                {
                    mem_fault = true;
                }
            }
            if (mem_fault)
            {

                // send_instruction(proc_id, INSTRUCTION_ERROR, 22, "cannot read from freed memory");
                // return;
            }
            send_instruction(proc_id, INSTRUCTION_OK);

            clear_read_buf();
            read_buffer_size = 0;
            pathfinder_read(a * 8, b * 8);
            // append instructions to instruction list
        }
        else if (inst_id == INSTRUCTION_ABORT)
        {
            curr_ins = 0;
            step_ins.clear();
            send_instruction(proc_id, INSTRUCTION_OK);
        }
        else if (inst_id == INSTRUCTION_PAUSE)
        {
            paused = true;
            Serial.println("got here");
            send_instruction(proc_id, INSTRUCTION_OK);
        }
        else if (inst_id == INSTRUCTION_RESUME)
        {
            paused = false;
            send_instruction(proc_id, INSTRUCTION_OK);
        }
        else if (inst_id == INSTRUCTION_RESTART)
        {
            curr_ins = 0;
            send_instruction(proc_id, INSTRUCTION_OK);
        }
        else if (inst_id == INSTRUCTION_RESULT)
        {
            send_instruction(proc_id, INSTRUCTION_RESULT, read_buffer_size + 1, read_buffer);
        }
        else if (inst_id == INSTRUCTION_STATUS)
        {
            char temp_data[5];
            if (paused)
            {
                temp_data[0] = (uint8_t)status_paused;
            }
            else if (step_ins.empty() || curr_ins >= step_ins.size())
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
        }
        else if (inst_id == INSTRUCTION_ERROR)
        {
            // wtf am i realistically supposed to do if the pc sends an error?
            // for now lets just ignore it and hope it gets handeled pc-side or the system is restated
        }
        else if (inst_id == INSTRUCTION_MEMORY_MAP)
        {
            char bits[MEMORY_SIZE] = {0};
            for (int i = 0; i < MEMORY_SIZE; i++)
            {
                bits[i] = (char)memory[i];
            }
            send_instruction(proc_id, INSTRUCTION_MEMORY_MAP, MEMORY_SIZE, bits);
            // send_instruction(30, INSTRUCTION_MEMORY_MAP);
        }
        else if (inst_id == INSTRUCTION_WALK)
        {
            send_instruction(proc_id, INSTRUCTION_OK);
            // int motor = serial_data_buff[0];
            // int goal_post = serial_data_buff[1];
            if (data_size != 2)
            {
                send_instruction(1, INSTRUCTION_ERROR, 19, "wrong datasize size");
            }
            if (serial_data_buff[0] == 1)
            {
                stepper1.enable();
                stepper1.walk(serial_data_buff[1] - stepper1.position);
                stepper1.disable();
                stepper1.position = serial_data_buff[1];
            }
            if (serial_data_buff[0] == 2)
            {
                stepper2.enable();
                stepper2.walk(serial_data_buff[1] - stepper2.position);
                stepper2.disable();
                stepper2.position = serial_data_buff[1];
            }

            if (serial_data_buff[0] == 3)
            {
                stepper4.enable();
                stepper4.walk(serial_data_buff[1] - stepper4.position);
                stepper4.disable();
                stepper4.position = serial_data_buff[1];
            }
        }
        else if (inst_id == INSTRUCTION_EDGE)
        {
            // send_instruction(proc_id, INSTRUCTION_OK);
            stepper1.enable();
            stepper1.walk(30); // reverse revolution
            stepper1.disable();

            stepper4.enable();
            while (!is_motor3_edging())
            {
                stepper4.rotate(-0.2);
            }
            stepper4.disable();
            delay(500);
            stepper4.enable();
            stepper4.walk(50); // reverse revolution
            stepper4.disable();
            stepper4.position = 50;

            // delay(500);

            stepper2.enable();
            while (!is_motor2_edging())
            {
                stepper2.rotate(-0.2);
            }
            stepper2.disable();
            delay(500);
            stepper2.enable();
            stepper2.walk(50); // reverse revolution
            stepper2.disable();
            stepper2.position = 50;

            // delay(500);

            stepper1.enable();
            while (!is_motor1_edging())
            {
                stepper1.rotate(-0.2);
            }
            stepper1.disable();
            delay(500);
            stepper1.enable();
            stepper1.walk(10); // reverse revolution
            stepper1.disable();
            stepper1.position = 10;

            curr_ins++;
            break;
        }

        else
        {
            send_instruction(proc_id, INSTRUCTION_ERROR, 18, "uknown instruction");
            // Serial.print("instruction:");
            // Serial.print(proc_id2, DEC);
            // Serial.print(proc_id, DEC);
            // Serial.println(inst_id, DEC);
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
    //     stepper4.walk(5);
    // }
    // stepper4.disable();

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

    // stepper4.enable();
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
    //     stepper4.walk(-5);
    // }
    // stepper4.disable();

    // while (!is_motor3_edging())
    // {
    //   Serial.print(">cstm4: ");
    //   Serial.println(is_motor2_edging());
    //   stepper2.walk(-1);
    // }
    // while ()
    //   Serial.print(">cstm4: ");
    // Serial.println(is_motor2_edging());
    // delay(500);
    // stepper2.enable();
    // stepper2.walk(220); // reverse revolution
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
    // // stepper4.setMicrostep(32);
    // // stepper4.setMicrostep(32);
    // // while (true)
    // // {

    // delay(500);
    // stepper4.enable();
    // stepper4.walk(50); // reverse revolution
    // stepper4.disable();
    // delay(500);
    // stepper4.enable();
    // stepper4.walk(-50); // reverse revolution
    // stepper4.disable();

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
    //       stepper1.walk(-1);
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
    //       stepper1.walk(1);
    //     }
    //   }
    //   Serial.println("finished rotating");
    //   digitalWrite(SLEEP1_PIN, LOW);
    //   // stepper1.rotate(360);
    // }

    // ! actual code!!!

    Serial.print(">alanog1: ");
    Serial.println(edger.getState());
    ask_for_instructions(100);
    if (millis() < last_start_byte + 1000)
    {
        digitalWrite(LED_13, true);
    }

    if (step_ins.empty() || paused || curr_ins >= step_ins.size())
    {
        return;
    }

    // Stepper *t = &steppers[step_ins.at(curr_ins).motor];
    switch (step_ins.at(curr_ins).type)
    {

    case wait:
        // Serial.println("waiting");
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
        if (step_ins.at(curr_ins).motor == 1 && (step_ins.at(curr_ins).goal_post < 0 || step_ins.at(curr_ins).goal_post > 200))
        {
            send_instruction(1, INSTRUCTION_ERROR, 28, "stepper 1 goal out of bounds");
            return;
        }
        if (step_ins.at(curr_ins).motor == 2 && (step_ins.at(curr_ins).goal_post < 0 || step_ins.at(curr_ins).goal_post > 195))
        {
            send_instruction(1, INSTRUCTION_ERROR, 28, "stepper 2 goal out of bounds");
            return;
        }
        if (step_ins.at(curr_ins).motor == 3 && (step_ins.at(curr_ins).goal_post < 0 || step_ins.at(curr_ins).goal_post > 220))
        {
            send_instruction(1, INSTRUCTION_ERROR, 28, "stepper 3 goal out of bounds");
            return;
        }

        // Serial.print(step_ins.size());
        // Serial.print(", ");
        // Serial.print(curr_ins);
        // Serial.print(", ");
        // Serial.print(step_ins.at(curr_ins).motor);
        // Serial.print(", ");
        // Serial.println(step_ins.at(curr_ins).goal_post);
        if (step_ins.at(curr_ins).motor == 1)
        {
            stepper1.enable();
            stepper1.walk(step_ins.at(curr_ins).goal_post - stepper1.position);
            stepper1.disable();
            stepper1.position = step_ins.at(curr_ins).goal_post;
        }
        if (step_ins.at(curr_ins).motor == 2)
        {
            stepper2.enable();
            stepper2.walk(step_ins.at(curr_ins).goal_post - stepper2.position);
            stepper2.disable();
            stepper2.position = step_ins.at(curr_ins).goal_post;
        }

        if (step_ins.at(curr_ins).motor == 3)
        {
            stepper4.enable();
            stepper4.walk(step_ins.at(curr_ins).goal_post - stepper4.position);
            stepper4.disable();
            stepper4.position = step_ins.at(curr_ins).goal_post;
        }

        curr_ins++;
        break;

    case read_sensor:
        if (step_ins.at(curr_ins).read_pos / 8 > read_buffer_size)
        {
            read_buffer_size = step_ins.at(curr_ins).read_pos / 8;
        }
        bitWrite(read_buffer[step_ins.at(curr_ins).read_pos / 8], step_ins.at(curr_ins).read_pos % 8, digitalRead(LDR_PIN));
        Serial.print("dot id ");
        Serial.print(step_ins.at(curr_ins).read_pos);
        Serial.print(" in state: ");
        Serial.println(digitalRead(LDR_PIN));
        curr_ins++;
        break;

    case edge:
        stepper1.enable();
        stepper1.walk(30); // reverse revolution
        stepper1.disable();

        stepper4.enable();
        while (!is_motor3_edging())
        {
            stepper4.rotate(-0.2);
        }
        stepper4.disable();
        delay(500);
        stepper4.enable();
        stepper4.walk(50); // reverse revolution
        stepper4.disable();
        stepper4.position = 50;

        // delay(500);

        stepper2.enable();
        while (!is_motor2_edging())
        {
            stepper2.rotate(-0.2);
        }
        stepper2.disable();
        delay(500);
        stepper2.enable();
        stepper2.walk(50); // reverse revolution
        stepper2.disable();
        stepper2.position = 50;

        // delay(500);

        stepper1.enable();
        while (!is_motor1_edging())
        {
            stepper1.rotate(-0.2);
        }
        stepper1.disable();
        delay(500);
        stepper1.enable();
        stepper1.walk(10); // reverse revolution
        stepper1.disable();
        stepper1.position = 10;

        curr_ins++;
        break;
    }

    // while (true)
    // {
    //     stepper4.enable();
    //     // stepper4.walk(30); // reverse revolution
    //     delay(1000);
    //     // stepper4.walk(-30); // reverse revolution
    //     stepper4.disable();
    //     delay(1000);
    // }
    // ! end of actual code

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
    edger.setDebounceTime(50);

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
    pinMode(RESET2_PIN, OUTPUT);z

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

    // stepper1.enable();
    // stepper1.walk(30); // reverse revolution
    // stepper1.disable();
    // stepper2.enable();
    // stepper2.walk(30); // reverse revolution
    // stepper2.disable();

    // stepper2.enable();
    // stepper2.walk(30); // reverse revolution
    // stepper2.disable();

    // stepper4.enable();
    // stepper4.walk(65); // reverse revolution
    // stepper4.disable();

    // stepper3.enable();
    // stepper3.walk(30); // reverse revolution
    // stepper3.disable();

    // stepper1.enable();
    // stepper1.walk(30); // reverse revolution
    // stepper1.disable();

    stepper_instruction inst;
    inst.type = edge;
    step_ins.push_back(inst);
    curr_ins = 0;

    // stepper4.enable();
    // stepper4.rotate(360); // reverse revolution
    // stepper4.disable();
    // stepper3.position = 10;

    // stepper4.enable();
    // stepper4.rotate(360);
    // stepper4.disable();
    // delay(500);

    // pathfinder_write(0, 139);
    // vector<pair<int, int>> p = {make_pair(3, 2), make_pair(-1, -1), make_pair(4, 2), make_pair(4, 3), make_pair(3, 3), make_pair(3, 4), make_pair(3, 3), make_pair(4, 3), make_pair(4, 4), make_pair(5, 4), make_pair(4, 4), make_pair(4, 3), make_pair(5, 3), make_pair(5, 2), make_pair(-2, -2), make_pair(4, 11), make_pair(-1, -1), make_pair(2, 11), make_pair(2, 13), make_pair(4, 13), make_pair(4, 11), make_pair(6, 11), make_pair(6, 13), make_pair(4, 13), make_pair(4, 11), make_pair(3, 11), make_pair(3, 6), make_pair(5, 6), make_pair(5, 11), make_pair(-2, -2)};
    // pathfinder(p);

    // stepper_instruction inst1;
    // inst1.type = walk_stepper;
    // inst1.motor = 1;
    // inst1.goal_post = 17;
    // step_ins.insert(step_ins.begin() + 3, inst1);

    // stepper_instruction inst2;
    // inst2.type = walk_stepper;
    // inst2.motor = 1;
    // inst2.goal_post = 35;
    // step_ins.insert(step_ins.end(), inst2);
    // step_ins.insert(step_ins.end(), inst1);

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