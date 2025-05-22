#include "tasks.h"

#include "timebase.h"

/**
 * @brief Executes scheduled tasks based on their intervals.
 *
 * This function checks each task in the task table and runs it if:
 * - Its interval is zero (run immediately or continuously), or
 * - The elapsed time since its last run is greater than or equal to its interval.
 *
 * After running a task, the function updates the task's last run timestamp.
 */
void tasksRun(void) {
  uint32_t now = millis();

  for (uint8_t i = 0; i < NUM_TASKS; ++i) {
    Task *t = &task_table[i];
    if (t->interval_ms == 0 || (now - t->last_run_ms) >= t->interval_ms) {
      t->func();
      t->last_run_ms = now;
    }
  }
}
