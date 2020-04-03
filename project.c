#include <HAL.h>
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include "Strings.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>



int run=0;
int train=0;
int suspended=0;

xTaskHandle xRedHandle;
xTaskHandle xGreenHandle;
xTaskHandle xTrainHandle;
xTaskHandle xHazardHandle;


static void vTask1( void *pvParameters );
static void vTask2( void *pvParameters );
static void vTask3( void *pvParameters );


void vApplicationIdleHook(){
		for( ;; )
	{
if(train==1){
	
	train=2;
	vTaskResume(xTrainHandle);

	}

	if(train==3){
	
	train=0;
	suspended=0;
	  vTaskSuspend(xTrainHandle);
		
		vTaskResume(xRedHandle);
		vTaskResume(xGreenHandle);

	}
	
}
	}


void PortF_Init(void);
// SystemCoreClock is referenced in FreeRTOSConfig.h.  It will be defined here and assigned
//	the bus clock frequency when the hardware is initialized.
uint32_t SystemCoreClock;

static SemaphoreHandle_t switchSemaphore_;
static uint8_t switchPressed_;

// These are the period task creation parameters.

void PortF_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x00000020; // activate clock for port F
  GPIO_PORTF_DIR_R |= 0x0E;  
  GPIO_PORTF_DEN_R |= 0x0E;     // enable digital I/O on PF3-PF1 
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_DEN_R = 0x0E;          // 7) enable digital pins PF3-PF1        
}

// Call when there is a catastophic problem.
static void ErrHandler(void)
{
	while (1) {}
}




void SwitchHandler(uint32_t pinMap)
{
	
	// Disable interrupts for both switches.
	GPIO_DisarmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));
	
	// Record the switch state for the switch task.
	switchPressed_ = (uint8_t)pinMap;
	
	//if hazard mode-> raise the hazard mode flag without switching the task
	
	if (switchPressed_ & PIN4) {
		run=1;
		}

	//if train mode-> wake up the train task and force switching from the ISR to it
		//TODO implement the second train switch 
	else{
		
		//vTaskPrioritySet(xTrainHandle, 5);

		if(train==2){
		train=3;
		GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));

		}
		else if(train==0 ){
		train=1;
		GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));

		}
		/*if(train==1){
			train=2;
		}
	
		// This will attempt a wake the higher priority SwitchTask and continue
	//	execution there.
		
		else{
			if(train==3){
				vTaskResume(xTrainHandle);}*/

	/*BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// Give the semaphore and unblock the SwitchTask.
	xSemaphoreGiveFromISR(switchSemaphore_, &xHigherPriorityTaskWoken);

	// If the SwitchTask was successfully woken, then yield execution to it
	//	and go there now (instead of changing context to another task).
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}*/
	}
	
	
	}
	




void vSwitchTask(void *pvParameters)
{
	
	for( ;; )
	{

		TickType_t xLastWakeTime;
	  xLastWakeTime = xTaskGetTickCount();
		
		vTaskSuspend(xGreenHandle);
		vTaskSuspend(xRedHandle);
		suspended=1;


		GPIO_PORTF_DATA_R = 0xe;    
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 5000 ) );	
		
		
if(suspended){	
	  suspended=0;
	  train=0;
		vTaskResume(xRedHandle);
		vTaskResume(xGreenHandle);
		vTaskSuspend(xTrainHandle);
}




		/*// Block until the switch ISR has signaled a switch press event...
		BaseType_t taken = xSemaphoreTake(switchSemaphore_, portMAX_DELAY);
		
			// Rearm interrupts for both switches.
			GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));
		
		if (taken == pdPASS) {
		
			TickType_t xLastWakeTime = xTaskGetTickCount();

		GPIO_PORTF_DATA_R = 0xe;    
		vTaskDelay( pdMS_TO_TICKS( 6000 ) );	
			}*/
		}
		
	
	
		
		}
	
	

	static void vTask1( void *pvParameters )
{


	
	for( ;; )
	{
		TickType_t xLastWakeTime;
	  xLastWakeTime = xTaskGetTickCount();
		
		vTaskSuspend(xGreenHandle);		


		GPIO_PORTF_DATA_R = 0x02;    
		//delaym(5);
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 5000 ) );	
		vTaskResume(xGreenHandle);

		
		if(run){
		vTaskPrioritySet(NULL, 1);
		vTaskPrioritySet(xHazardHandle, 4);
		vTaskResume(xHazardHandle);
		}

		else
			{
		vTaskPrioritySet(NULL, 2);
		vTaskPrioritySet(xGreenHandle, 3);}
		


}
}


static void vTask2( void *pvParameters )
{

	for( ;; )
	{
	  TickType_t xLastWakeTime;
	  xLastWakeTime = xTaskGetTickCount();
		vTaskSuspend(xRedHandle);
    GPIO_PORTF_DATA_R = 0x08;
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 2500 ) );	
		vTaskResume(xRedHandle);


		
				
		if(run){
		vTaskPrioritySet(NULL, 1);
		vTaskPrioritySet(xHazardHandle, 4);
		vTaskResume(xHazardHandle);

		}

		else
			{
		vTaskPrioritySet(NULL, 2);
		vTaskPrioritySet(xRedHandle, 3);}



	}
}



	static void vTask3( void *pvParameters )
{


	for( ;; )
	{
		run=0;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
		
	GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));

	vTaskSuspend(xRedHandle);
	vTaskSuspend(xGreenHandle);


	GPIO_PORTF_DATA_R = 0x04;
	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 3000 ) );	
	
	vTaskResume(xRedHandle);
	vTaskResume(xGreenHandle);
	vTaskSuspend(xHazardHandle);
	
	}
		}      



static int InitHardware(void)
{
	 PortF_Init();

	__disable_irq();
	
	PLL_Init80MHz();

	// Must store the frequency in SystemCoreClock for FreeRTOS to use.
	SystemCoreClock = PLL_GetBusClockFreq();

	// These are the digital outputs for the LEDs.
	//GPIO_EnableDO(PORTC, PIN5 | PIN6 | PIN7, DRIVE_2MA, PULL_DOWN);
	//GPIO_InitPort(PORTF);
	
	// These are the digital intputs for the onboard buttons.
	GPIO_EnableDI(PORTF, PIN0 | PIN4, PULL_UP);
	
	// Enable interrupts for SW1 & SW2.
	GPIO_EnableInterrupt(&PINDEF(PORTF, PIN0), 7, INT_TRIGGER_FALLING_EDGE, SwitchHandler);
	GPIO_EnableInterrupt(&PINDEF(PORTF, PIN4), 7, INT_TRIGGER_FALLING_EDGE, SwitchHandler);
	

	__enable_irq();
	
	return 0;
	
}


int main()
{

	InitHardware();
	
	// Create the semaphore that the switch ISR and task will synchronize on.
	switchSemaphore_ = xSemaphoreCreateBinary();
	
	if (switchSemaphore_ != NULL) {
		
	xTaskCreate(vSwitchTask, (const portCHAR *)"train", 256, NULL, 5, &xTrainHandle);
	xTaskCreate( vTask1, (const portCHAR *)"red light", 256, NULL,2 , &xRedHandle );
	xTaskCreate( vTask2, (const portCHAR *)"green light", 256, NULL, 3, &xGreenHandle );
	xTaskCreate( vTask3, (const portCHAR *)"hazard", 256, NULL, 1, &xHazardHandle );
	
	vTaskSuspend(xTrainHandle);
	vTaskSuspend(xHazardHandle);


	vTaskStartScheduler();
	
	}	
		
	for (;;);

}
