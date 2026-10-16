#include <stdbool.h>
#include <stdint.h>

#include "sleep.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xstatus.h"
#include "xtime_l.h"

#include "app/adas/adas.h"

int main(void)
{
    bool running;

    Xil_ICacheEnable();
    Xil_DCacheEnable();

    running = App_Init();
    while (running)
    {
        App_PollIo(App_NowMs());
        App_ServiceCnn(App_NowMs());
        /* No inference wait here. Lane IRQs remain enabled during CNN work. */
        App_PollIo(App_NowMs());
        usleep((unsigned int)APP_IO_POLL_PERIOD_US);
    }

    if (s_can_initialized)
    {
        (void)AdasCanApp_SetState(&s_adas, ADAS_ECU_FAULT);
    }
    xil_printf("Application stopped in FAULT state\r\n");

    for (;;)
    {
        if (s_can_initialized)
        {
            App_PollIo(App_NowMs());
        }
        usleep((unsigned int)APP_IO_POLL_PERIOD_US);
    }
}
