// Standard includes
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include "stdlib.h"

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "hw_mcspi.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "spi.h"
#include "udma.h"
#include "math.h"
#include "timer.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "udma_if.h"

#include "pin_mux_config.h"

#define APPLICATION_VERSION     "1.1.1"

#define MASTER_MODE 0
#define SPI_IF_BIT_RATE 500000
#define TR_BUFF_SIZE 2
#define REFRACTORY_PERIOD 250 //heart refractory period in ms
//fft
#define SAMPLING_RATE 472.1//500
#define PI 3.14159265358979323846264338327950288
#define q 12 //2^q points
#define N (1<<q) //N-point fft

#define PREP_MESSAGE "READY TO RECEIVE DATA"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

typedef enum{
    Step1,
    Step2,
    Step3,
    Step4
}StepperState;

typedef float real;
typedef struct { real Re; real Im; } complex;

//Transmit and receive buffers for SPI
static unsigned char rxBuffer[TR_BUFF_SIZE];
static unsigned int txBuffIndex;

unsigned short temp;
unsigned char temp2;


//Averaging filter
float newAvg=0;
StepperState step = Step1;
//FFT
double adc_arr[N];
static int adc_arr_idx=0;
complex adc_vec[N], temp_fft[N];
double f;
int ix=0;
//double frequency = 0;
double max;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
void LEDBlinkyRoutine();
void BinCount(void);
void Stepper(void);
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

void FFT(complex *v, int n, complex *tmp){
    if(n>1){
        int k,m; complex z, w, *vo, *ve;
        ve = tmp; vo = tmp + n/2;
        for (k=0; k<n/2;k++){
            ve[k] = v[2*k];
            vo[k] = v[2*k+1];
        }
        FFT(ve,n/2,v);
        FFT(vo,n/2,v);
        for(m=0; m<n/2;m++){
            w.Re = cos(2 * PI*m / (double)n);
            w.Im = -sin(2 * PI*m / (double)n);
            z.Re = w.Re*vo[m].Re - w.Im*vo[m].Im;
            z.Im = w.Re*vo[m].Im + w.Im*vo[m].Re;
            v[m].Re = ve[m].Re + z.Re;
            v[m].Im = ve[m].Im + z.Im;
            v[m + n / 2].Re = ve[m].Re - z.Re;
            v[m + n / 2].Im = ve[m].Im - z.Im;
            v[m].Re /= 2;
            v[m].Im /=2;
            v[m + n / 2].Re /= 2;
            v[m + n / 2].Im /= 2;
        }
    }
    return;
}

void IFFT(complex *v, int n, complex *tmp){
    if (n>1){
        int k,m; complex z, w, *vo, *ve;
        ve = tmp; vo = tmp + n/2;
        for (k=0; k<n/2;k++){
            ve[k] = v[2*k];
            vo[k] = v[2*k+1];
        }
        IFFT(ve,n/2,v);
        IFFT(vo,n/2,v);
        for(m=0; m<n/2;m++){
            w.Re = cos(2* PI*m / (float)n);
            w.Im = sin(2 * PI*m / (float)n);
            z.Re = w.Re*vo[m].Re - w.Im*vo[m].Im;
            z.Im = w.Re*vo[m].Im + w.Im*vo[m].Re;
            v[m].Re = ve[m].Re + z.Re;
            v[m].Im = ve[m].Im + z.Im;
            v[m + n / 2].Re = ve[m].Re - z.Re;
            v[m + n / 2].Im = ve[m].Im - z.Im;
        }
    }
    return;
}

float get_freq(complex adc_d[],complex scratch[]){
    double frequency = 0;
    FFT(adc_d,N,scratch);
    max = sqrt((adc_d[1].Re*adc_d[1].Re)+(adc_d[1].Im*adc_d[1].Im));//adc_d[0] is the DC component
    //FFT2(adc_d,N);
    int idx;
    int abs_complex=0;
    for(idx=1;idx<N;idx++){
        abs_complex=sqrt((adc_d[idx].Re*adc_d[idx].Re)+(adc_d[idx].Im*adc_d[idx].Im));
        if(abs_complex>max){
            max=abs_complex;
            frequency = (double)idx * (double)SAMPLING_RATE / (double)N;
        }
    }
    return frequency;
}

void MasterConfig(){
    //Set uC for master mode
    SPIReset(GSPI_BASE);
    SPIConfigSetExpClk(GSPI_BASE, PRCMPeripheralClockGet(PRCM_GSPI),
                       SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                                            (SPI_SW_CTRL_CS |
                                            SPI_4PIN_MODE |
                                            SPI_TURBO_OFF |
                                            SPI_CS_ACTIVELOW |
                                            SPI_WL_8));

    SPIEnable(GSPI_BASE);
    SPICSEnable(GSPI_BASE);

}

int exp_moving_avg(float in, float avg, float alpha){
    double tmp;
    tmp = (double)in * alpha + (double)avg * (1-alpha);
    return (float)tmp;
}

void receive_process(){

    SPITransfer(GSPI_BASE,0,rxBuffer,2,
                SPI_CS_ENABLE|SPI_CS_DISABLE);
    //Report("%i\n\r",rxBuffer[1]);
    temp = ((rxBuffer[1]>>3) | ((rxBuffer[0] & 0x1F)<<5));
    //Report("%i\t\n\r",(temp));
    //temp = (int)temp*(int)temp;
    newAvg = exp_moving_avg(temp,newAvg,0.1);
    temp = newAvg;//-650;
    adc_arr[adc_arr_idx] = (double)temp;
    adc_vec[adc_arr_idx].Re = (double)(((temp)));//Square adc value to separate
    adc_vec[adc_arr_idx].Im = 0;
    Report("%f\r\n",adc_vec[adc_arr_idx].Re);

    if(adc_arr_idx==N-1){
        //f = get_freq(adc_vec, temp_fft);
        adc_arr_idx=0;
        //f*=2;
        //Report("\n%.0f BPM\n\r",f*60);
        //UtilsDelay(40000000);
    }
    else{
        adc_arr_idx++;
    }
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


void HeartBeat(){
    switch(step){
        default:
        case Step1:
            //step1
            GPIOPinWrite(GPIOA0_BASE, 1<<6, 1<<6);
            GPIOPinWrite(GPIOA1_BASE, 1<<2, 0<<2);
            GPIOPinWrite(GPIOA0_BASE, 1<<7, 1<<7);
            GPIOPinWrite(GPIOA0_BASE, 1<<5, 0<<5);
            step = Step2;
            break;
        case Step2:
            //step2
            GPIOPinWrite(GPIOA0_BASE, 1<<6, 1<<6);
            GPIOPinWrite(GPIOA1_BASE, 1<<2, 0<<2);
            GPIOPinWrite(GPIOA0_BASE, 1<<7, 0<<7);
            GPIOPinWrite(GPIOA0_BASE, 1<<5, 1<<5);
            step = Step3;
            break;
        case Step3:
            //step3
            GPIOPinWrite(GPIOA0_BASE, 1<<6, 0<<6);
            GPIOPinWrite(GPIOA1_BASE, 1<<2, 1<<2);
            GPIOPinWrite(GPIOA0_BASE, 1<<7, 0<<7);
            GPIOPinWrite(GPIOA0_BASE, 1<<5, 1<<5);
            step = Step4;
            break;
        case Step4:
            //step4
            GPIOPinWrite(GPIOA0_BASE, 1<<6, 0<<6);
            GPIOPinWrite(GPIOA1_BASE, 1<<2, 1<<2);
            GPIOPinWrite(GPIOA0_BASE, 1<<7, 1<<7);
            GPIOPinWrite(GPIOA0_BASE, 1<<5, 0<<5);
            step = Step1;
            break;

    }
    /*if(pulse_rate>=600 && pulse_rate<2000){
        UtilsDelay(pulse_delay);
    }*/
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//! This function  
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int
main()

{
    //
    // Initialize Board configurations
    //
    BoardInit();
    
    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();

    PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    InitTerm();
    //ClearTerm();
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    
    PRCMPeripheralReset(PRCM_GSPI);//Reset Peripheral

    //Enable Direct Memory Access
    //UDMAInit();

    MasterConfig();



    /*complex v[N], v1[N], scratch[N];
    int k;


    for (k = 0; k<N; k++) {
        v[k].Re = .125*cos(2 * PI*k / (double)N);
        v[k].Im = sin(2 * PI*k / (double)N);
    }
    Message("\nCHECK0\n");

    for(k=0;k<N;k++){
        Report("Orig %d: %f + j%f\n\r",k,v[k].Re,v[k].Im);
    }
    FFT(v, N, scratch);
    for(k=0;k<N;k++){
        Report("FFT %d: %f + j%f\n\r",k,v[k].Re,v[k].Im);
    }
    IFFT(v, N, scratch);
    for(k=0;k<N;k++){
        Report("IFFT %d: %f + j%f\n\r",k,v[k].Re,v[k].Im);
    }*/

    while(1){
        receive_process();
        //HeartBeat();
        //printf("%d\n",txBuffIndex++);
    }

    //
    // Start the LEDBlinkyRoutine
    //
    //LEDBlinkyRoutine();
    //BinCount();
    //Stepper();
    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************