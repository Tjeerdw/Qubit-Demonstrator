#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NeoPixelBus.h>
#include <AccelStepper.h>

#include "config.h"

TaskHandle_t motorTask;

OneWire oneWire(ONE_WIRE_BUS); // OneWire data wire connected to GPIO19
DallasTemperature sensors(&oneWire);

DeviceAddress sensor1 = {0x28, 0xDA, 0xF2, 0xDE, 0xA, 0x0, 0x0, 0x4E};

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(LED_COUNT, LED_PIN);

RgbColor red(LED_COLOR_SATURATION, 0, 0);
RgbColor green(0, LED_COLOR_SATURATION, 0);
RgbColor blue(0, 0, LED_COLOR_SATURATION);
RgbColor white(LED_COLOR_SATURATION);
RgbColor black(0);

AccelStepper stepper1(AccelStepper::DRIVER, STPA_STEP, STPA_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, STPB_STEP, STPB_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, STPC_STEP, STPC_DIR);

// const double coeffs[3][3] = {
//     {-.8660254, 0., -.5},
//     {.4330127, -.75, -.5},
//     {.4330127, .75, -.5}
// };

const double coeffs[3][3] = {
    {0.81915204,  -0.,         0.57357644},
    {-0.40957602, -0.70940648, 0.57357644},
    {-0.40957602,  0.70940648, 0.57357644}
};

double pos[3] {0.};
const double HALF_ROTATION(9450 / coeffs[2][1]);
const double MAX_SPEED(20000.0);
const double ACCELERATION(2000.0);
int iteration = 0;
bool busy = false;
bool ready = false;
bool write_done = false;
double update[3] = {0.};

void update_positions(double (&update)[3], const double (&u)[3], const double theta)
{
    update[0] = (coeffs[0][0] * u[0] + coeffs[0][1] * u[1] + coeffs[0][2] * u[2]) * theta;
    update[1] = (coeffs[1][0] * u[0] + coeffs[1][1] * u[1] + coeffs[1][2] * u[2]) * theta;
    update[2] = (coeffs[2][0] * u[0] + coeffs[2][1] * u[1] + coeffs[2][2] * u[2]) * theta;
    ready = true;
}

void motorCall(void * parameter)
{
    for(;;){
        if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0)
        {
            double max_update = max(max(abs(update[0]), abs(update[1])), abs(update[2]));
            if (busy)
            {
                busy = false;
                write_done = true;
            }
            if (ready)
            {
                busy = true;
                
                delay(30);

                pos[0] += update[0];
                pos[1] += update[1];
                pos[2] += update[2];

                ready = false;
                
                stepper1.setAcceleration(update[2] / max_update * ACCELERATION);
                stepper2.setAcceleration(update[1] / max_update * ACCELERATION);
                stepper3.setAcceleration(update[0] / max_update * ACCELERATION);

                stepper1.setMaxSpeed(update[2] / max_update * MAX_SPEED);
                stepper2.setMaxSpeed(update[1] / max_update * MAX_SPEED);
                stepper3.setMaxSpeed(update[0] / max_update * MAX_SPEED);

                stepper1.moveTo(pos[2]);
                stepper2.moveTo(pos[1]);
                stepper3.moveTo(pos[0]);

                update[0] = 0;
                update[1] = 0;
                update[2] = 0;
            }
        }
        else
        {
            stepper1.run();
            stepper2.run();
            stepper3.run();
        }
    }
}

void setup()
{
    Serial.begin(115200);
    sensors.begin();

    strip.Begin();
    strip.Show();

    stepper1.setMaxSpeed(20000.0);
    stepper1.setAcceleration(2000.0);
    stepper1.moveTo(0);

    stepper2.setMaxSpeed(20000.0);
    stepper2.setAcceleration(2000.0);
    stepper2.moveTo(0);

    stepper3.setMaxSpeed(20000.0);
    stepper3.setAcceleration(2000.0);
    stepper3.moveTo(0);

    pinMode(STP_EN, OUTPUT);

    pinMode(STP_MS1, OUTPUT);
    pinMode(STP_MS2, OUTPUT);
    pinMode(STP_MS3, OUTPUT);

    digitalWrite(STP_EN, LOW);

    digitalWrite(STP_MS1, HIGH); //configure for 1/16 microstep
    digitalWrite(STP_MS2, HIGH); //used to set to 1/8 microstep due to software bug.
    digitalWrite(STP_MS3, HIGH);

    strip.SetPixelColor(0, green);
    strip.SetPixelColor(1, green);
    strip.SetPixelColor(2, green);
    strip.SetPixelColor(3, green);
    strip.Show();

    disableCore0WDT(); //I disable the core becasue i dont have the WDT reset functions working yet
    xTaskCreatePinnedToCore(motorCall, "motorTask", 1000, NULL, 1, &motorTask, 0);
}

double read_param()
{
    // almost, we use the edge case 128/-128 differently
    int num = -1;
    while (num == -1) {num = Serial.read();}
    return ((double)num) / 127 - 1.0;
}

void loop()
{
    // sensors.requestTemperatures();
    // Serial.print(sensors.getTempC(sensor1));
    // Serial.println(" *C Sensor ");

    if (write_done)
    {
        Serial.write((int)'d');
        write_done = false;
    }

    int data = 0;
    data = Serial.read();
    
    // This might be useful for debugging purposes in the future
    /*if (data != -1)
    {
        Serial.print("Received data:");
        Serial.println(data);
    }*/

    switch (data)
    {
        case (int)'r':
            strip.SetPixelColor(0, red);
            strip.SetPixelColor(1, red);
            strip.SetPixelColor(2, red);
            strip.SetPixelColor(3, red);
            strip.Show();
            break;
        case (int)'g':
            strip.SetPixelColor(0, green);
            strip.SetPixelColor(1, green);
            strip.SetPixelColor(2, green);
            strip.SetPixelColor(3, green);
            strip.Show();
            break;
    }
    if (!busy)
    {
        switch(data)
        {
            case (int)'x':
                update_positions(update, (double[3]){1,0,0}, HALF_ROTATION);
                break;
            case (int)'y':
                update_positions(update, (double[3]){0,1,0}, HALF_ROTATION);
                break;
            case (int)'z':
                update_positions(update, (double[3]){0,0,1}, HALF_ROTATION);
                break;
            case (int)'h':
                update_positions(update, (double[3]){.707,0,.707}, HALF_ROTATION);
                break;
            case (int)'s':
                update_positions(update, (double[3]){0,0,1}, .5*HALF_ROTATION);
                break;
            case (int)'S':
                update_positions(update, (double[3]){0,0,1}, -.5*HALF_ROTATION);
                break;
            case (int)'t':
                update_positions(update, (double[3]){0,0,1}, .25*HALF_ROTATION);
                break;
            case (int)'T':
                update_positions(update, (double[3]){0,0,1}, -.25*HALF_ROTATION);
                break;
            case (int)'u':
                double x = read_param();
                double y = read_param();
                double z = read_param();
                double angle = read_param();
                update_positions(update, (double[3]){x,y,z}, angle * HALF_ROTATION);
                break;
        }
    }
    delay(100);
}
