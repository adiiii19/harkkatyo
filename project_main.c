/* C Standard library */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"


/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];
Char morseReadTaskStack[STACKSIZE];


//Tilakone

enum state { WAITING=1, DATA_READY };
enum state programState = WAITING;


// Global variables
#define ROWS 30
#define COLS 6
double ambientLight = -1000.0;
float mpuValues[ROWS][COLS];
char movementData[60];
char morseCode[100];
char allMessages[5][200];
int sorter;
int programsState2 = 0;
//float maxValues[6];


static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;
static PIN_Handle MpuPin;
static PIN_State MpuPinState;

static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};

PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    // Vilkuta jompaa kumpaa ledia
    uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );

}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {

    // UARTin alustus: 9600,8n1

    UART_Handle uart;
    UART_Params uartParams;


    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode=UART_MODE_BLOCKING;
    uartParams.baudRate = 9600;
    uartParams.dataLength = UART_LEN_8;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.stopBits = UART_STOP_ONE;

    uart = UART_open(Board_UART0, &uartParams);
       if (uart == NULL) {
           System_abort("Error opening the UART in uartTask");
       }

    while (1) {

        //Data=ready, tulostaa debug ikkunaan
        char dbg_msg[200];
        if (programState2 == 1) {

            sprintf(dbg_msg, "%s\n\r", allMessages[0]);
            System_printf("%s\n", dbg_msg);
            System_flush();
            UART_write(uart, dbg_msg, strlen(dbg_msg));
            programState2 = 0;
            }



        // sanity check
        //System_printf("uartTask\n");
        //System_flush();

        //once per second
        Task_sleep(500000 / Clock_tickPeriod);
    }
}

Void sensorTaskFxn(UArg arg0, UArg arg1) {

    while (1) {
            float ax, ay, az, gx, gy, gz;
            I2C_Handle i2cMPU;
            I2C_Params i2cMPUParams;
            //I2C_Handle      i2c;
            //I2C_Params      i2cParams;

            //sanity check
            System_printf("datacollect\n");
            System_flush();

            I2C_Params_init(&i2cMPUParams);
            i2cMPUParams.bitRate = I2C_400kHz;

            i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

            PIN_setOutputValue(MpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

            Task_sleep(100000 / Clock_tickPeriod);
            System_printf("MPU9250: Power ON\n");
            System_flush();

            i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
            if (i2cMPU == NULL) {
                System_abort("Error Initializing I2CMPU\n");
            }

            System_printf("MPU9250: Setup and calibration...\n");
            System_flush();

            mpu9250_setup(&i2cMPU);

            System_printf("MPU9250: Setup and calibration OK\n");
            System_flush();

            // Start time before data gathering
            int32_t startTime = Clock_getTicks(); // Get the current tick count

            int i, j;



            System_printf("Data gathering start\n");
            System_flush();

            for (i = 0; i <= 30; i++) {

                mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);


                mpuValues[i][0] = ax;
                mpuValues[i][1] = ay;
                mpuValues[i][2] = az;
                mpuValues[i][3] = gx;
                mpuValues[i][4] = gy;
                mpuValues[i][5] = gz;

                Task_sleep(100000 / Clock_tickPeriod);

            }
            System_printf("Data gathering end\n");
            System_flush();

            for (j = 0; j <= 30; j++) {

                sprintf(movementData, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                        mpuValues[j][0], mpuValues[j][1], mpuValues[j][2], mpuValues[j][3], mpuValues[j][4], mpuValues[j][5]);
                System_printf(movementData);
                System_flush();

            }
            System_printf("\n");
            System_flush();

            // MPU close i2c
            I2C_close(i2cMPU);
            //MPU power off
            PIN_setOutputValue(MpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

            System_printf("MPU9250: Power off\n");
            System_flush();

            Task_sleep(1000000 / Clock_tickPeriod);

        }
}

/* Comparison function to sort rows based on gyrox (column index 3) */
int compare(const void* a, const void* b) {
    // Extract gyrox values from both rows and compare
    float valueA = ((float*)a)[sorter];
    float valueB = ((float*)b)[sorter];

    // Sort in descending order for highest gyrox values first
    if (valueA > valueB) return -1;
    else if (valueA < valueB) return 1;
    else return 0;
}

/* Sort function that uses qsort and compare to sort mpuValues by gyrox */
void sortByParam(float mpuValues[ROWS][COLS]) {
    qsort(mpuValues, ROWS, sizeof(mpuValues[0]), compare);
}


    void morseReadTask(UArg arg0, UArg arg1) {

        Float ax, ay, az, gx, gy, gz;

        I2C_Handle i2cMPU;
        I2C_Params i2cMPUParams;

        Int currentTime = Clock_getTicks();

        //sanity check
        System_printf("morsetask\n");
        System_flush();

        I2C_Params_init(&i2cMPUParams);
        i2cMPUParams.bitRate = I2C_400kHz;

        i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

        PIN_setOutputValue(MpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

        //Task_sleep(100000 / Clock_tickPeriod);
        System_printf("MPU9250: Power ON\n");
        System_flush();

        i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
        if (i2cMPU == NULL) {
            System_abort("Error Initializing I2CMPU\n");
        }

        System_printf("MPU9250: Setup and calibration...\n");
        System_flush();

        mpu9250_setup(&i2cMPU);

        System_printf("MPU9250: Setup and calibration OK\n");
        System_flush();

        System_printf("Morse Gathering start\n");
        System_flush();

        int movementDetected = 0;
        int i=0;
        int j=0;
        int msg=0;

        while(1){
            for (i = 0; i <= 30; i++) {

                mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

                mpuValues[i][0] = ax;
                mpuValues[i][1] = ay;
                mpuValues[i][2] = az;
                mpuValues[i][3] = gx;
                mpuValues[i][4] = gy;
                mpuValues[i][5] = gz;

                Task_sleep(100000 / Clock_tickPeriod);

                }
                sorter = 3;
                sortByParam(mpuValues);

                if (mpuValues[0][3]>10) {
                    if (movementDetected==0) {
                        movementDetected=1;
                    }
                //System_printf("Suuri gyro\n");
                //System_flush();
                }
                if(movementDetected == 1){
                    //pöytäliike, tarkastellaan gyroz ja acceleration y
                    /*if (fabs(gx) > 150){
                        strcat(morseCode, ".");
                        j++;
                    }*/
                    //pyörähdysliike, tarkastellaan gyro x
                    if (mpuValues[0][3]>100){
                       strcat(morseCode, "-");
                       j++;
                    }
                    //System_printf("liikettä\n");
                    //System_flush();
                }
                if(movementDetected==0){
                    if(morseCode[j-1]=='.'||morseCode[j-1]=='-'){
                        strcat(morseCode, " ");
                    }
                    else if(morseCode[j-1]==' '&&morseCode[j-2]!=' '){
                        strcat(morseCode, " ");
                        }
                    else if(morseCode[j-1]==' '&&morseCode[j-2]==' '){
                        strcat(morseCode, " ");
                        strcpy(allMessages[msg], morseCode);
                        msg++;
                        //memset(morseCode, '\0', sizeof(morseCode));
                        System_printf("viesti valmis");
                        programState2 = 1;
                    }
                    j++;
                }

                System_printf("%s\n", morseCode);
                System_flush();
                movementDetected = 0;



        }

        /*
        // MPU close i2c
        I2C_close(i2cMPU);
        //MPU power off
        PIN_setOutputValue(MpuPin,Board_MPU_POWER, Board_MPU_POWER_OFF);

        System_printf("MPU9250: Power off\n");
        System_flush();

        Task_sleep(1000000 / Clock_tickPeriod);*/


    }



Int main(void) {

    // Task variables
    //Task_Handle sensorTaskHandle;
    //Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;
    Task_Handle morseReadTaskHandle;
    Task_Params morseReadTaskParams;

    // Initialize board
    Board_initGeneral();

    Board_initI2C();

    ledHandle = PIN_open( &ledState, ledConfig );
    if(!ledHandle) {
       System_abort("Error initializing LED pin\n");
    }

    // Painonappi käyttöön ohjelmassa
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pin\n");
    }

    // Painonapille keskeytyksen käsittellijä
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }

    MpuPin = PIN_open(&MpuPinState, MpuPinConfig);
        if (MpuPin == NULL) {
            System_abort("Pin open failed!");
        }



    /* Task */


    /*Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }*/

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=1;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&morseReadTaskParams);
    morseReadTaskParams.stackSize = STACKSIZE;
    morseReadTaskParams.stack = &morseReadTaskStack;
    morseReadTaskParams.priority=2;
    morseReadTaskHandle = Task_create(morseReadTask, &morseReadTaskParams, NULL);
    if (morseReadTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    //System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
