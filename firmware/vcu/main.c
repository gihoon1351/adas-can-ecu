#include "app/app_vcu.h"

/*
 * Initialization
 * APP_VCU_Init clears the runtime state, configures the safety-domain monitor
 * and watchdog, initializes CAN/GPIO/timers, connects the interrupt handlers,
 * and starts the timers. It returns failure if interrupt setup fails; main then
 * stops instead of running without periodic scheduling or safety monitoring.
 *
 * Operation and service priority
 * This is a cooperative superloop, not an RTOS priority system. Timer ISRs
 * advance millis() and set flags; APP_VCU_Run performs the actual work in this
 * fixed order on each pass:
 * 1. Handle the safety-domain IRQ and CAN bus-off recovery first.
 * 2. Process queued CAN RX before checking heartbeat, motor confirmation,
 *    startup readiness, and faults, so the checks can use the newest frames.
 * 3. Run ADAS and motor control only when the drive gate is open and no
 *    emergency stop is latched.
 * 4. Attempt at most one queued CAN transmission, bounding bus work per pass.
 * 5. Service heartbeat timeout/TX and watchdog flags; print diagnostics last.
 * A latched emergency stop is serviced at the end of every APP_VCU_Run pass.
 *
 * Startup interlock
 * The drive gate remains closed until every ECU has sent a heartbeat. The VCU
 * then queues a global fault reset, waits for all heartbeats to become healthy,
 * requests motor alignment, and waits for fresh status showing both motors
 * aligned. This prevents motion commands from reaching an unready system.
 *
 * Fault and emergency stop
 * Heartbeat loss, command-confirmation timeout, CAN/ADAS queue overflow, ADAS
 * result timeout, or bus-off latches the emergency stop and disables vehicle
 * output. The VCU retries the E-stop command and requests both motor stops.
 * If both motors do not confirm the safe state before the deadline, the
 * watchdog is forced to reset the system. A persistent bus-off also requests
 * watchdog reset after its recovery timeout.
 *
 * Heartbeat and WDT
 * TIM1 runs at 1 ms: CAN TX is released every tick, while RX, fault checks, and
 * control are released every 10 ms at staggered phases. TIM2 runs at 10 ms and
 * staggers heartbeat timeout checks, VCU heartbeat TX, watchdog kicks, and
 * diagnostics so each task runs every 100 ms. The watchdog is not kicked after
 * a reset has been requested.
 *
 * Why this ordering
 * Safety and fault checks precede control so a newly latched fault blocks
 * motor commands in the same pass. Short ISRs keep interrupt latency bounded;
 * rate-limited CAN TX and lower-priority diagnostics leave the main loop
 * responsive to safety and control work.
 */
int main(void)
{

	if (APP_VCU_Init() != 0) {
		return -1;
	}

	while (1) {
		APP_VCU_Run();
	}
	return 0;
}
