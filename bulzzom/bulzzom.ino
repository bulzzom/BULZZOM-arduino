#include <FreeRTOS_AVR.h>
#include <Servo.h>
#include <SoftwareSerial.h>

/* Macro */
#define SEC(x) ((unsigned long)(x)*configTICK_RATE_HZ)
#define MS(x)  (((unsigned long)(x)*configTICK_RATE_HZ)/1000L)
#define BZ_QUEUE_SIZE 5
#define BZ_MAX_SERVO  1
#define BZ_MAX_QUEUE  3

/* Enumerations */
typedef enum {
  BZ_TIMER,
  BZ_ALARM,
  BZ_SERVO
} BZ_TYPE;

/* Tasks */
static void bz_ReceiveTask(void* arg);
static void bz_TimerTask(void* arg);
static void bz_ServoTask(void* arg);

/* Kernel Objects */
QueueHandle_t bz_MsgQueue[BZ_MAX_QUEUE];

/* Define structures */
struct bz_servo {
  char num;
  int angle;
};
struct bz_timer {
  int second;
  struct bz_servo servo;
};

/* Global variable */
SoftwareSerial bluetooth(2, 3); // (BT_RXD, BT_TXD)
Servo bz_servoArr[BZ_MAX_SERVO];
const char *sendComplete = "OK\n";

/* Function prototype */

void setup() {
  short i;
  Serial.begin(9600);
  bluetooth.begin(9600);

  bz_MsgQueue[BZ_TIMER] = xQueueCreate(BZ_QUEUE_SIZE, sizeof(struct bz_timer));
  bz_MsgQueue[BZ_ALARM] = xQueueCreate(BZ_QUEUE_SIZE, sizeof(struct bz_timer));
  bz_MsgQueue[BZ_SERVO] = xQueueCreate(BZ_QUEUE_SIZE, sizeof(struct bz_servo));

  for(i=0; i<BZ_MAX_SERVO; i++)
    bz_servoArr[i].attach(7+i);
  
  xTaskCreate(bz_ReceiveTask, NULL, 200, NULL, 1, NULL);
  xTaskCreate(bz_ServoTask, NULL, 200, NULL, 2, NULL);
  xTaskCreate(bz_TimerTask, NULL, 200, NULL, 3, NULL);

  vTaskStartScheduler();     // start scheduler
  for(;;) {
  }
}

void loop() {
  // Idle
  for(;;) {
  }
}

static void bz_ReceiveTask(void* arg) {
  for(;;) {
    if(bluetooth.available()) {
      switch(bluetooth.read()) {
        case 'S': // 'S'[char:num][int:angle]
          struct bz_servo newServo;
          
          while(!bluetooth.available());
          newServo.num = bluetooth.read();
          
          while(!bluetooth.available());
          newServo.angle = bluetooth.parseInt();
          
          xQueueSendToBack(bz_MsgQueue[BZ_SERVO], &newServo, 0);
          break;
        case 'T': // 'T'[char:num][char:on/off][int:second]
          struct bz_timer newTimer;
          
          while(!bluetooth.available());
          newTimer.servo.num = bluetooth.read();
          
          while(!bluetooth.available());
          switch(bluetooth.read()) {
          case 'N':
            newTimer.servo.angle = 160;
            break;
          case 'F':
            newTimer.servo.angle = 50;
            break;
          }

          while(!bluetooth.available());
          newTimer.second = bluetooth.parseInt();
          
          xQueueSendToBack(bz_MsgQueue[BZ_TIMER], &newTimer, 0);
          break;
        default:
          break;
      }
    }
  }
}

static void bz_TimerTask(void* arg) {
  struct bz_timer newTimer;

  for(;;) {
    if(xQueueReceive(bz_MsgQueue[BZ_TIMER], &newTimer, portMAX_DELAY)) {
      vTaskDelay(SEC(newTimer.second));
      if(newTimer.servo.num <= 'C' && newTimer.servo.num >= 'A') {
        bz_servoArr[newTimer.servo.num - 'A'].write(newTimer.servo.angle);
        
        bluetooth.write(sendComplete);
        vTaskDelay(SEC(2));
        bz_servoArr[newTimer.servo.num - 'A'].write(90);
      } else if (newTimer.servo.num == 'D') {
        for(int j=0; j<BZ_MAX_SERVO; j++)
              bz_servoArr[j].write(newTimer.servo.angle);

        bluetooth.write(sendComplete);
        vTaskDelay(SEC(2));
        for(int j=0; j<BZ_MAX_SERVO; j++)
              bz_servoArr[j].write(90);
      }
    }
  }
}

static void bz_ServoTask(void* arg) {
  struct bz_servo newServo;

  for(;;) {
    if(xQueueReceive(bz_MsgQueue[BZ_SERVO], &newServo, portMAX_DELAY)) {
      if(newServo.num <= 'C' && newServo.num >= 'A') {
        bz_servoArr[newServo.num - 'A'].write(newServo.angle);

        bluetooth.write(sendComplete);
        vTaskDelay(SEC(2));
        bz_servoArr[newServo.num - 'A'].write(90);
      } else if (newServo.num == 'D') {
        for(int j=0; j<BZ_MAX_SERVO; j++)
          bz_servoArr[j].write(newServo.angle);

        bluetooth.write(sendComplete);
        vTaskDelay(SEC(2));
        for(int j=0; j<BZ_MAX_SERVO; j++) {
          bz_servoArr[j].write(90);
        }
      }
    }
  }
}
