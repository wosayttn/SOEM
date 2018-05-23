/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <osal.h>
#include <time.h>

struct timeval
{
    uint32 tv_sec;     /*< Seconds elapsed since the Epoch (Jan 1, 1970) */
    uint32 tv_usec;    /*< Microseconds elapsed since last second boundary */
};

#define  timercmp(a, b, CMP)                                \
  (((a)->tv_sec == (b)->tv_sec) ?                           \
   ((a)->tv_usec CMP (b)->tv_usec) :                        \
   ((a)->tv_sec CMP (b)->tv_sec))
#define  timeradd(a, b, result)                             \
  do {                                                      \
    (result)->tv_sec = (a)->tv_sec + (b)->tv_sec;           \
    (result)->tv_usec = (a)->tv_usec + (b)->tv_usec;        \
    if ((result)->tv_usec >= 1000000)                       \
    {                                                       \
       ++(result)->tv_sec;                                  \
       (result)->tv_usec -= 1000000;                        \
    }                                                       \
  } while (0)
#define  timersub(a, b, result)                             \
  do {                                                      \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;           \
    (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;        \
    if ((result)->tv_usec < 0) {                            \
      --(result)->tv_sec;                                   \
      (result)->tv_usec += 1000000;                         \
    }                                                       \
  } while (0)

#define USECS_PER_SEC   1000000
#define USECS_PER_TICK  (USECS_PER_SEC / CFG_TICKS_PER_SECOND)

static uint64_t GetCurrentTimeInMS(void)
{
	//It is system boot time.
	TimeOut_t sTimeOut;
	uint64_t u64CurrentTicks;
	vTaskSetTimeOutState(&sTimeOut);
	u64CurrentTicks = ((uint64_t)sTimeOut.xOverflowCount * portMAX_DELAY) + sTimeOut.xTimeOnEntering;
	return u64CurrentTicks * portTICK_RATE_MS;
}

void freertos_udelay (uint32_t us)
{
   uint64_t u64start_ms, u64stop_ms;
   TickType_t ticks = (us / 1000);
	
   u64start_ms = GetCurrentTimeInMS();
	 vTaskDelay ( ticks / portTICK_PERIOD_MS);
	 u64stop_ms = GetCurrentTimeInMS();
}

static int gettimeofday(struct timeval *tp, void *tzp)
{
   uint64_t CurrentTimeInMS = GetCurrentTimeInMS();
   if ( tp == NULL ) return -1;
	 tp->tv_sec  = CurrentTimeInMS/1000;
	 tp->tv_usec = (CurrentTimeInMS % 1000) * 1000;
   return 0;
}


int osal_usleep (uint32 usec)
{
	// minimal delay is 1ms.
	if ( usec < 1000 )
		usec = 1000;
	freertos_udelay(usec);	
	return 0;
}

static int osal_gettimeofday(void *tv, void *tz)
{
   (void)tz;       /* Not used */
   if ( tv == NULL ) return -1;
   return gettimeofday(tv, tz);	
}

ec_timet osal_current_time(void)
{
   ec_timet return_value={0,0};
	
   osal_gettimeofday((struct timeval *)&return_value, NULL);
	
	 return return_value;
}

void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
   if (end->usec < start->usec) {
      diff->sec = end->sec - start->sec - 1;
      diff->usec = end->usec + 1000000 - start->usec;
   }
   else {
      diff->sec = end->sec - start->sec;
      diff->usec = end->usec - start->usec;
   }
}

void osal_timer_start(osal_timert * self, uint32 timeout_usec)
{
   struct timeval start_time;
   struct timeval timeout;
   struct timeval stop_time;

   osal_gettimeofday(&start_time, 0);

   timeout.tv_sec = timeout_usec / USECS_PER_SEC;
   timeout.tv_usec = timeout_usec % USECS_PER_SEC;
   timeradd(&start_time, &timeout, &stop_time);

   self->stop_time.sec = stop_time.tv_sec;
   self->stop_time.usec = stop_time.tv_usec;
}

boolean osal_timer_is_expired (osal_timert * self)
{
   struct timeval current_time;
   struct timeval stop_time;
   int is_not_yet_expired;

   osal_gettimeofday(&current_time, 0);
   stop_time.tv_sec = self->stop_time.sec;
   stop_time.tv_usec = self->stop_time.usec;
   is_not_yet_expired = timercmp(&current_time, &stop_time, <);

   return is_not_yet_expired == FALSE;
}

void *osal_malloc(size_t size)
{
   return malloc(size);
}

void osal_free(void *ptr)
{
   free(ptr);
}

#define DEF_OSAL_TRHEAD_PRIO_DEFAULT 	(configMAX_PRIORITIES-2)
int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{	
  BaseType_t res = xTaskCreate((TaskFunction_t)func, "THREAD_DEFAULT", stacksize, param, DEF_OSAL_TRHEAD_PRIO_DEFAULT, &thandle);
	if(res != pdPASS) {
		printf("Failed to create task-DEFAULT!");
		return 0;
	}
  return 1;
}

#define DEF_OSAL_TRHEAD_PRIO_RT		   	(configMAX_PRIORITIES-1)
int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
    BaseType_t res = xTaskCreate((TaskFunction_t)func, "THREAD_RT", stacksize, param, DEF_OSAL_TRHEAD_PRIO_RT, &thandle);
    if(res != pdPASS) {
        printf("Failed to create task-RT!");
				return 0;
    }
		return 1;
}
