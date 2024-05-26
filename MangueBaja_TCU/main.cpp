#include "defs.h"
#include "mbed.h"
#include "stats_report.h"
/* Instances Libraries */
#include "CANMsg.h"
#include "MLX90614.h"
/* User Libraries */
#include "rear_defs.h"

#define default_addr (0x00)

//#define FRONT_WHEEL
#define REAR_WHEEL

/* Communication protocols */
CAN can(PB_8, PB_9, CAN_BPS_1000K);       // RD, TD, Frequency
Serial serial(PA_2, PA_3, 115200);  // TX, RX, Baudrate
I2C i2c(PB_7, PB_6);                // SDA, SCl

/* I/O pins */
InterruptIn freq_sensor(PB_1, PullNone);
AnalogIn ReadTempMotor(PA_0);
//AnalogIn ReadLevel(PA_1);
AnalogIn ReadVoltage(PA_4);                
AnalogIn ReadSystemCurrent(PB_0); 
PwmOut servo(PA_6);
DigitalOut led(PC_13);
/* Debug pins */
//PwmOut signal(PA_7);
DigitalOut db(PB_11);

/* Instances Variables */
MLX90614 mlx(&i2c);

/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
CircularBuffer<state_t, BUFFER_SIZE> state_buffer;
Ticker ticker200mHz;
Ticker ticker250mHz;
Ticker ticker500mHz;
Ticker ticker1Hz;
Ticker ticker5Hz;

/* Debug variables */
Timer t;
bool buffer_full = false;
/* Global variables */
FIR filter(0.5, 0.5);
state_t current_state = IDLE_ST;
packet_t data;
//float Calculate_VacsI0 = 0.0;
bool switch_clicked = false;
uint8_t pulse_counter = 0;
uint8_t switch_state = 0x00;
//uint16_t SignalVacsI0;
uint16_t speed_display = 0;
uint16_t speed_filt = 0;              // 2by
uint64_t current_period = 0, last_count = 0, last_acq = 0;
float V_termistor = 0;
float speed_hz = 0;

/* Interrupt handlers */
void canHandler();
/* Interrupt services routine */
void canISR();
void frequencyCounterISR();
void ticker200mHzISR();
void ticker250mHzISR();
void ticker500mHzISR();
void ticker1HzISR();
void ticker5HzISR();
/* General functions*/
void initPWM();
void setupInterrupts();
void filterMessage(CANMsg msg);
float CVT_Temperature();
//float Level_Moving_Average();
float Voltage_moving_average();
float SystemCurrent_moving_average();
void writeServo(uint8_t MODE);

/* CAN Variables */
uint8_t temp_motor = 0;               // 1by
//uint16_t fuel = 0;                    // 2by
uint8_t MeasureCVTtemperature = 0;    // 1by
uint16_t speed = 0;
float MeasureVoltage = 0.0;           // 4by
uint8_t SOC = 0;                      // 1by
float MeasureSystemCurrent = 0.0;     // 4by

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    initPWM();
    setupInterrupts();

    while (true)
    {
        if(state_buffer.full()) 
        {
            buffer_full = true;
            //led = 0;
            state_buffer.pop(current_state);
        } else {
            //led = 1;
            buffer_full = false;
            if(!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }


        switch(current_state) 
        {
            case IDLE_ST:
                //serial.printf("idle\r\n");
                //Thread::wait(1);
                break;

            case TEMP_MOTOR_ST:
                //serial.printf("motor\r\n");

                V_termistor = ADCVoltageLimit*ReadTempMotor.read();

                //calc1 = (float)115.5*(exp(-0.02187*(V_termistor*R_TERM)/(ADCVoltageLimit - V_termistor)));
                //calc2 = (float)85.97*(exp(-0.00146*(V_termistor*R_TERM)/(ADCVoltageLimit - V_termistor)));
                temp_motor = (float)115.5*(exp(-0.02187*(V_termistor*R_TERM)/(ADCVoltageLimit - V_termistor)))
                           + (float)85.97*(exp(-0.00146*(V_termistor*R_TERM)/(ADCVoltageLimit - V_termistor)));

                /* Send Motor Temperature data */
                txMsg.clear(TEMPERATURE_ID);
                txMsg << (temp_motor==0 ? 1 : temp_motor);
                can.write(txMsg);

                break;

            case TEMP_CVT_ST:
                //serial.printf("cvt\r\n");

                MeasureCVTtemperature = (uint8_t)CVT_Temperature(); 
                //MeasureCVTtemperature = 90;

                /* Send CVT Temperature message */
                txMsg.clear(CVT_ID);
                txMsg << MeasureCVTtemperature;
                can.write(txMsg);
                //if(can.write(txMsg))
                //    led = !led;
                
                break;

            case FUEL_ST:
                //serial.printf("fuel\r\n");
                //fuel = 100;

                //fuel = (uint16_t)Level_Moving_Average();
                //fuel = 100;

                ///* Send Fuel data */
                //txMsg.clear(FUEL_ID);
                //txMsg << fuel;
                //can.write(txMsg); 

                break;

            case SPEED_ST:
                //serial.printf("speed\r\n");

                freq_sensor.fall(NULL);         // disable interrupt

                if (current_period!=0)
                {
                    speed_hz = 1000000*((float)(pulse_counter)/current_period);    //calculates frequency in Hz
                } 
                
                else 
                {
                    speed_hz = 0;
                }

                #ifdef FRONT_WHEEL
                    speed_display = ((float)(3.6*PI*WHEEL_DIAMETER*speed_hz)/WHEEL_HOLES_NUMBER_FRONT);    // make conversion hz to km/h
                #endif

                #ifdef REAR_WHEEL
                    speed_display = ((float)(3.6*PI*WHEEL_DIAMETER*speed_hz)/WHEEL_HOLES_NUMBER_REAR);    // make conversion hz to km/h
                #endif

                //speed_radio = ((float)((speed_display)/60.0)*65535);
                speed_filt = (uint16_t)filter.filt(speed_display);

                if(data.rpm!=0)
                    speed = speed_filt - 10;
                else
                    speed = speed_filt;

                /* Send Speed data */
                txMsg.clear(SPEED_ID);
                txMsg << speed;
                can.write(txMsg);
                //if(can.write(txMsg))
                //    led = !led;

                /* re-init the counter */
                pulse_counter = 0;                          
                current_period = 0;                         //|-> reset pulses related variables
                last_count = t.read_us();        
                freq_sensor.fall(&frequencyCounterISR);     // enable interrupt                

                break;    

            case VOLTAGE_ST:
                //serial.printf("voltage\r\n");
                
                MeasureVoltage = Voltage_moving_average();
            
                /*
                a = -8E-8*pow(MeasureVoltage,4);
                b = 3E-5*pow(MeasureVoltage,3);
                c = -0.0026*pow(MeasureVoltage,2);
                d = -0.4058*MeasureVoltage;
                SOC = (uint8_t)(a + b + c + d + 100);
                */

                SOC = (uint8_t)(((MeasureVoltage - 9.6)/3)*100); /* 12.6 - 9.6 = 3 */

                //Led = !Led;
                //SOC = 100;

                /* Send Voltage message */
                //MeasureVoltage100 = 100 * (uint8_t ) MeasureVoltage; 
                txMsg.clear(VOLTAGE_ID);
                txMsg << MeasureVoltage;
                //can.write(txMsg);
                if(can.write(txMsg)) 
                {
                    /* Send SOC(State of Charge) message if voltage successfully */
                    //led = !led;

                    txMsg.clear(SOC_ID);
                    txMsg << (SOC > 100 ? 5 : SOC);
                    can.write(txMsg);
                }

                break;

            case SYSTEM_CURRENT_ST:
                //serial.printf("current\r\n");
                //SignalVacsI0 = ReadSystemCurrent.read_u16();
                //Calculate_VacsI0 = (SignalVacsI0*(ADCVoltageLimit/65535.0));
                // Calculate_VacsI0 = SignalVacsI0 ;
                MeasureSystemCurrent = SystemCurrent_moving_average();
                //MeasureSystemCurrent = MeasureSystemCurrent * 1.23885;
                
                /* Send current message */
                txMsg.clear(CURRENT_ID);
                txMsg << MeasureSystemCurrent;
                can.write(txMsg);
                //if(can.write(txMsg))
                //    led = !led;


                break;

            case THROTTLE_ST:
                //serial.printf("throttle\r\n");

                if(switch_clicked)
                {
                    writeServo(switch_state);
                    switch_clicked = false;
                }

                break;

            case DEBUG_ST:
                //serial.printf("Debug state\r\n");
                //serial.printf("Temperature Motor = %d\r\n", temp_motor);
                //serial.printf("CVT Temperature = %d\r\n", MeasureCVTtemperature);
                //serial.printf("Speed = %d\r\n", speed_filt);
                //serial.printf("Voltage = %f\r\n", MeasureVoltage);
                //serial.printf("SOC = %d\r\n", SOC);
                //serial.printf("Current = %f\r\n", MeasureSystemCurrent);
                //serial.printf("switch state = %d\r\n", switch_state);
                //serial.printf("\n\n\n");
                break;
        }
    }
}

/* General functions */
void initPWM()
{
    servo.period_ms(20);                        // set signal frequency to 50Hz
    servo.write(0);                             // disables servo
    //signal.period_ms(320);                      // set signal frequency to 1/0.032Hz
    //signal.write(0.5f);                         // dutycycle 50%
}

void setupInterrupts()
{
    /* General Interrupts */
    can.attach(&canISR, CAN::RxIrq);
    freq_sensor.fall(&frequencyCounterISR); // enable interrupt

    /* Tickers */
    ticker1Hz.attach(&ticker1HzISR, 1.0);
    ticker5Hz.attach(&ticker5HzISR, 0.2);
    //ticker500mHz.attach(&ticker500mHzISR, 2);
}

void filterMessage(CANMsg msg)
{
    led = !led;

    if(msg.id == THROTTLE_ID)
    {
        switch_clicked = true;
        msg >> switch_state;
        //state_buffer.reset();
        state_buffer.push(THROTTLE_ST);
    }

    if(msg.id == RPM_ID)
    {
        msg >> data.rpm;
    }
}

float CVT_Temperature()
{
    int i, j;
    float AverageObjectTemp, AverageEnviromentTemp = 0.0;
    float temp_amb, med_amb, x_amb, x_obj, med_obj;
    char ucdata_write[2]; // teste comunicação i2c

    if(!i2c.write((default_addr|0x00), ucdata_write, 1, 0)) // Check for ACK from i2c Device
    {
        for(j = 0; j < (sample); j++) 
        { 
            for(i = 0; i < (sample) ; i++) 
            {              
                // temp_amb = mlx.read_temp(0);
                //temp_obj = mlx.read_temp(1);
                x_obj += mlx.read_temp(1);
                med_obj = x_obj/(float)sample;

                if(med_obj > AverageObjectTemp)
                    AverageObjectTemp = med_obj;
            }
        }
    } 
    
    else 
    {
        AverageObjectTemp = 0;
    }

    //return value;
    return AverageObjectTemp/(float)sample; // I don't know why we need divide by sample, but works.
}

/*float Level_Moving_Average()
{
    float AverageLevel=0;
    float Vout, P;
    float med_level, x_level;

    for(uint8_t i = 0; i < LevelSample; i++)
    {
        // for(uint8_t j = 0; j < LevelSample; j++)
        // {
        //     Vout = (ReadLevel.read_u16()*ADCVoltageLimit)/65535.0; 
        //     if(Vout < SensorADClimit)
        //     {
                
        //         P = ((7171*Vout)-5566)*DENSITY;

        //         x_level += P;
        //     } 
             
        //     else 
        //     {
        //         med_level = 0;
        //     }                
        // }
        med_level = x_level/(float)LevelSample;

        if(med_level > AverageLevel)
            AverageLevel = med_level;
    }

    return AverageLevel/(float)LevelSample; // I don't know why we need divide by sample, but works.
}*/

float Voltage_moving_average()
{
    int i, j;
    float value, aux, ADCvoltage , InputVoltage, AverageVoltage = 0.0;
    //float Calibration_Factor = 0.987;
    //float Calibration_Factor = 0.715;
    float Calibration_Factor = 1.0;
    float R1_Value = 30000.0;               // VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
    float R2_Value = 7500.0;                // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO

    for(j = 0; j < (sample); j++) 
    { 
        for(i = 0; i < (sample) ; i++) 
        {
            InputVoltage = (float)((ReadVoltage.read_u16() * ADCVoltageLimit) / 65535.0);
            ADCvoltage = Calibration_Factor*(InputVoltage / (R2_Value/(R1_Value + R2_Value)));
            aux += ADCvoltage;
        }
        value = aux/(float)sample;
        
        if(value > AverageVoltage) 
            AverageVoltage = value;
    }

    //return value;
    return AverageVoltage/(float)(sample); // I don't know why we need divide by sample, but works.
}

float SystemCurrent_moving_average()
{   
    int i, j;
    float value, ux, InputSystemCurrent, AverageSystemCurrent, ADCSystemCurrent = 0.0;
    // float Current_Calibration_Factor = 1.746;
    float Current_Calibration_Factor = 1.631;
    float VacsI0 = 1.558008;

    for(j = 0; j < (sample); j++)
    { 
        for(i = 0; i < (sample) ; i++) 
        {
            InputSystemCurrent = (float)(ReadSystemCurrent.read_u16() * (ADCVoltageLimit / 65535.0));
            ADCSystemCurrent = (VacsI0 - InputSystemCurrent) / 0.185;
            ux += ADCSystemCurrent;
        }       
        value = (ux * Current_Calibration_Factor)/((float)sample*4);
        //value = (ux * 1.746)  / ((double)sample*4);
    }

    //return value;
    return value/(float)sample; // I don't know why we need divide by sample, but works.
}

void writeServo(uint8_t MODE)
{
    switch(MODE) 
    {
        case MID_MODE:
            servo.pulsewidth_us(SERVO_MID);
            //data.flags &= ~(0x03); // reset run and choke flags
            break;
        case RUN_MODE:
            servo.pulsewidth_us(SERVO_RUN);
            //data.flags |= RUN_MODE;    // set run flag
            break;
        case CHOKE_MODE:
            servo.pulsewidth_us(SERVO_CHOKE);
            //data.flags |= CHOKE_MODE;    // set choke flag
            break;
        default:
            //serial.printf("Choke/run error\r\n");
            break;
    }
}

/* Interrupt services routine */
void canISR()
{
    CAN_IER &= ~CAN_IER_FMPIE0;                 // disable RX interrupt
    queue.call(&canHandler);                    // add canHandler() to events queue
}

void frequencyCounterISR()
{
    //db = !db;
    pulse_counter++;
    current_period += t.read_us() - last_count;
    last_count = t.read_us();        
}

void ticker200mHzISR()
{
    state_buffer.push(VOLTAGE_ST);
    //state_buffer.push(DEBUG_ST);
}

void ticker250mHzISR()
{
    state_buffer.push(TEMP_MOTOR_ST);
    state_buffer.push(TEMP_CVT_ST);
}

void ticker500mHzISR()
{
    state_buffer.push(SYSTEM_CURRENT_ST);
}

void ticker1HzISR()
{
    state_buffer.push(TEMP_MOTOR_ST);
    state_buffer.push(TEMP_CVT_ST);

    state_buffer.push(VOLTAGE_ST);
    //state_buffer.push(FUEL_ST);

    state_buffer.push(SYSTEM_CURRENT_ST);

    ticker200mHz.attach(&ticker200mHzISR, 5);
    ticker250mHz.attach(&ticker250mHzISR, 4);
    ticker500mHz.attach(&ticker500mHzISR, 2);

    ticker1Hz.detach();
}

void ticker5HzISR()
{
    state_buffer.push(SPEED_ST);
}

/* Interrupt handlers */
void canHandler()
{
    CANMsg rxMsg;

    can.read(rxMsg);
    filterMessage(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}
