/**
 * CG2271 Lab 6 - C.A.R.T.O.S. (Car And RTOS)
 * @author Justin Ng, Lim Hong Wei, CEG2 2017
 */

#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define STACK_SIZE 100
#define BUZZER_PIN 9
#define RED_PIN 10
#define FAST_PIN 7
#define MED_PIN 6
#define SLOW_PIN 5
#define PTTM_PIN 0
#define DEBOUNCE_DELAY 200

static unsigned long previousPressedTime0 = 0, previousPressedTime1 = 0;
static unsigned int currentSpeed = 0;
static unsigned int desiredSpeed = 0;
static unsigned int emergencyBrake = 0;

QueueHandle_t xMessageQueue;
QueueHandle_t xSafeDistanceQueue;

struct Message {
    int mDistance;
    int mCurrentSpeed;
    int mDesiredSpeed;
};

void updateLedAndBuzzer(void *p) {
	portTickType xRedLedBeginTime;
	for (;;) {
		switch (currentSpeed) {
		case 3:
			digitalWrite(FAST_PIN, HIGH);
			digitalWrite(MED_PIN, HIGH);
			digitalWrite(SLOW_PIN, HIGH);
			break;
		case 2:
			digitalWrite(FAST_PIN, LOW);
			digitalWrite(MED_PIN, HIGH);
			digitalWrite(SLOW_PIN, HIGH);
			break;
		case 1:
			digitalWrite(FAST_PIN, LOW);
			digitalWrite(MED_PIN, LOW);
			digitalWrite(SLOW_PIN, HIGH);
			break;
		case 0:
			digitalWrite(FAST_PIN, LOW);
			digitalWrite(MED_PIN, LOW);
			digitalWrite(SLOW_PIN, LOW);
		}
		tone(BUZZER_PIN, currentSpeed*500+31); // 31Hz is base tone, other speeds vary step 500Hz.
		if (emergencyBrake == 1) {
			xRedLedBeginTime = xTaskGetTickCount();
			digitalWrite(RED_PIN, HIGH);
			emergencyBrake = 2;
		} else if (emergencyBrake == 2 && xTaskGetTickCount() == xRedLedBeginTime+1000) {
			digitalWrite(RED_PIN, LOW);
			emergencyBrake = 0;
		}
		vTaskDelay(5);
	}
}

void readAndSendSafeDistanceAhead(void *p) {
	const portTickType xFrequency = 500;
	portTickType xLastWakeTime = 0;
	int safeDistance, distance;
	for (;;) {
		distance  = analogRead(PTTM_PIN);
		if (distance < 256) {
			safeDistance = 0;
		} else if (distance < 512) {
			safeDistance = 1;
		} else if (distance < 768) {
			safeDistance = 2;
		} else if (distance < 1024) {
			safeDistance = 3;
		}
		// Send safe distance to speed controller
		xQueueSendToBack(xSafeDistanceQueue, &safeDistance, (TickType_t) 1);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void sendToUart(void *p) {
	const portTickType xFrequency = 1000;
	portTickType xLastWakeTime = 0;
	for (;;) {
		struct Message *pxRxedMessage;
		if (xQueueReceive(xMessageQueue, &(pxRxedMessage), (TickType_t) 1) == pdTRUE) {
			Serial.print("C");
			Serial.print((int) pxRxedMessage->mCurrentSpeed);
			Serial.print(" D");
			Serial.print((int) pxRxedMessage->mDesiredSpeed);
			Serial.print(" d");
			Serial.println((int) pxRxedMessage->mDistance);
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void adjustSpeed(void *p) {
	for (;;) {
		int oldSpeed = currentSpeed;
		int safeDistance;
		if (xQueueReceive(xSafeDistanceQueue, &safeDistance, (TickType_t) 1) == pdTRUE) {
			currentSpeed = min(desiredSpeed, safeDistance);
			// Latter part ensures that brakelight not activated by button.
			if (currentSpeed < oldSpeed && safeDistance < desiredSpeed) {
				emergencyBrake = 1;
			}
			// Send mesage to UART
			Message message = {safeDistance+1, (int)currentSpeed, (int)desiredSpeed};
			struct Message *pxMessage;
			pxMessage = &message;
			xQueueSendToBack(xMessageQueue, (void*)&pxMessage, (TickType_t) 1);
		}
		vTaskDelay(5);
	}
}

static void increaseSpeed() {
	if (desiredSpeed < 3) {
		desiredSpeed++;
	}
}

static void decreaseSpeed() {
	if (desiredSpeed > 0) {
		desiredSpeed--;
	}
}

void int0ISR() {
	unsigned long currentTime = millis();
	if (currentTime - previousPressedTime0 > DEBOUNCE_DELAY) {
		increaseSpeed();
	}
	previousPressedTime0 = currentTime;
}

void int1ISR() {
	unsigned long currentTime = millis();
	if (currentTime - previousPressedTime1 > DEBOUNCE_DELAY) {
		decreaseSpeed();
	}
	previousPressedTime1 = currentTime;
}

void setup()
{
	Serial.begin(115200);
	attachInterrupt(1, int0ISR, RISING);
	attachInterrupt(0, int1ISR, RISING);
	pinMode(FAST_PIN, OUTPUT);
	pinMode(MED_PIN, OUTPUT);
	pinMode(SLOW_PIN, OUTPUT);
	pinMode(RED_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	xMessageQueue = xQueueCreate(3, sizeof(struct Message *)); 	// Create message queue storing pointers to Message structs.
	xSafeDistanceQueue = xQueueCreate(3, sizeof(int));
}

void loop() {
	xTaskCreate(updateLedAndBuzzer,           // Pointer to the task entry function
		     "UpdateSpeedLed",         // Task name
		     STACK_SIZE,      // Stack size
		     NULL,       // Pointer that will be used as parameter
		     4,               // Task priority
		    NULL);           // Used to pass back a handle by which the created task can be referenced.

	xTaskCreate(readAndSendSafeDistanceAhead,           // Pointer to the task entry function
		     "ReadDistanceAhead",         // Task name
		     STACK_SIZE,      // Stack size
		     NULL,       // Pointer that will be used as parameter
		     3,               // Task priority
		    NULL);           // Used to pass back a handle by which the created task can be referenced.

	xTaskCreate(adjustSpeed,           // Pointer to the task entry function
		     "AdjustSpeed",         // Task name
		     STACK_SIZE,      // Stack size
		     NULL,       // Pointer that will be used as parameter
		     2,               // Task priority
		    NULL);           // Used to pass back a handle by which the created task can be referenced.

	xTaskCreate(sendToUart,           // Pointer to the task entry function
		     "SendToUart",         // Task name
		     STACK_SIZE,      // Stack size
		     NULL,       // Pointer that will be used as parameter
		     1,               // Task priority
		    NULL);           // Used to pass back a handle by which the created task can be referenced.

	vTaskStartScheduler();
}
