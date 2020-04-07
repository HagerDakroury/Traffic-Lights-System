#include <HAL.h>
#include "tm4c123gh6pm.h"
#include <FreeRTOS.h>
#include <task.h>


int run=0;
int train=0;
int suspended=0;

xTaskHandle xRedHandle;
xTaskHandle xGreenHandle;
xTaskHandle xTrainHandle;
xTaskHandle xHazardHandle;

uint32_t SystemCoreClock;
static uint8_t switchPressed_;

static void vTask1( void *pvParameters );
static void vTask2( void *pvParameters );
static void vTask3( void *pvParameters );
void vSwitchTask(void *pvParameters);
void PortF_Init(void);



/*The idle application handler */
void vApplicationIdleHook(){
		for( ;; )
	{
		
/*The case where the train button is pressed, the train task is immediately resumed*/
if(train==1){
	
	train=2;
	vTaskResume(xTrainHandle);

	}

	
/*Hadling If the train task is citt off by the second switch and the hazard button was pressed during the train mode */
else if((train==3)& run){
		suspended=0;
	  train=0;
		vTaskSuspend(xTrainHandle);
		
		vTaskResume(xHazardHandle);
		vTaskResume(xRedHandle);
		vTaskResume(xGreenHandle);
	}

/*The case where the train second switch is pressed (indicating leaving before the safety period) */
else if(train==3){
	
	  train=0;
	  suspended=0;
	  vTaskSuspend(xTrainHandle);
		
		vTaskResume(xRedHandle);
		vTaskResume(xGreenHandle);

	}
	
}
	}

	


/*Initializing the needed pins*/
void PortF_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x00000020; // activate clock for port F
  GPIO_PORTF_DIR_R |= 0x0E;  
  GPIO_PORTF_DEN_R |= 0x0E;     // enable digital I/O on PF3-PF1 
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_DEN_R = 0x0E;          // 7) enable digital pins PF3-PF1        
}



void SwitchHandler(uint32_t pinMap)
{
	
  //Disable interrupts for both switches.
	GPIO_DisarmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));
	
	//Record the switch state for the switch task.
	switchPressed_ = (uint8_t)pinMap;
	
	//if hazard mode-> raise the hazard mode flag without switching the task
	
	if (switchPressed_ & PIN4) {
		run=1;
		GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));

		}

	else{
		//if train mode is aleady running and second train switch is pressed -> change te state of the flag to indicate the end of the task
		if(train==2){
		train=3;
		GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));

		}
		
    //if train mode requested-> raise the train flag		
		else if(train==0 ){
		train=1;
		GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));

		}
	}
	
	
	}
	



/*The train mode*/
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
		
/*checking if hazard mode is request, so we can jump right into it instead of the normal mode */
if(run){
		train=0;
		vTaskResume(xHazardHandle);
		vTaskSuspend(xTrainHandle);

		}
	
	/*If the train task didn't stop immediately when the switch is pressed, finish properly */
if(suspended){	
	  suspended=0;
	  train=0;
		vTaskResume(xRedHandle);
		vTaskResume(xGreenHandle);
		vTaskSuspend(xTrainHandle);
}

		}
		}
	
	
/*The North/South Junction*/
	static void vTask1( void *pvParameters )
{
	for( ;; )
	{
		TickType_t xLastWakeTime;
	  xLastWakeTime = xTaskGetTickCount();
		
		vTaskSuspend(xGreenHandle);		


		GPIO_PORTF_DATA_R = 0x02;    
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 5000 ) );	
		vTaskResume(xGreenHandle);

		
/*Checking if Hazard mode is on-> resume the hazard task
	else switch to the East/West task*/		

		if(run){
		vTaskResume(xHazardHandle);
		}

		else
			{
		vTaskPrioritySet(NULL, 2);
		vTaskPrioritySet(xGreenHandle, 3);}
}
}


/*The East/West junction*/
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

/*Checking if Hazard mode is on-> resume the hazard task
	else switch to the North/South task*/		
		if(run){
		vTaskResume(xHazardHandle);

		}

		else
			{
		vTaskPrioritySet(NULL, 2);
		vTaskPrioritySet(xRedHandle, 3);}



	}
}




/*Hazard mode*/
static void vTask3( void *pvParameters )
{

	for( ;; )
	{
		run=0;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
		

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

	//initialize all ports and interrupts
	InitHardware();
	
	//creating the 4 tasks and suspending the ones that need triggers (train,hazard)
	xTaskCreate(vSwitchTask, (const portCHAR *)"train", 256, NULL, 5, &xTrainHandle);
	xTaskCreate( vTask1, (const portCHAR *)"red light", 256, NULL,3 , &xRedHandle );
	xTaskCreate( vTask2, (const portCHAR *)"green light", 256, NULL, 2, &xGreenHandle );
	xTaskCreate( vTask3, (const portCHAR *)"hazard", 256, NULL, 4, &xHazardHandle );
	
	vTaskSuspend(xTrainHandle);
	vTaskSuspend(xHazardHandle);


	vTaskStartScheduler();
	
	
		
	for (;;);

}
