#ifndef TASKS_H_
#define TASKS_H_

#include <stdint.h>

/**
 * @brief Represents a scheduled task.
 *
 * Contains the function pointer to the task function, the interval between runs
 * in milliseconds, and the timestamp of the last execution.
 */
typedef struct {
  void (*func)(void);  ///< Pointer to the task function to execute.
  uint32_t
      interval_ms;  ///< Interval between runs in milliseconds. If zero, runs every call.
  uint32_t last_run_ms;  ///< Timestamp of the last run in milliseconds.
} Task;

/**
 * @brief External declaration of the task table array.
 *
 * Defined elsewhere using the DEFINE_TASKS macro.
 */
extern Task task_table[];

/**
 * @brief External declaration of the number of tasks.
 *
 * Defined elsewhere using the DEFINE_TASKS macro.
 */
extern const uint8_t NUM_TASKS;

/**
 * @brief Macro to define the task table and the number of tasks.
 *
 * Use this macro in exactly one .c file to instantiate the task list and
 * automatically calculate the number of tasks.
 *
 * Example usage:
 *   DEFINE_TASKS(
 *     {task1_func, 1000, 0},
 *     {task2_func, 500, 0}
 *   );
 *
 * @param ... List of Task initializers.
 */
#define DEFINE_TASKS(...)            \
  Task task_table[] = {__VA_ARGS__}; \
  const uint8_t NUM_TASKS = sizeof(task_table) / sizeof(task_table[0])

/**
 * @brief Runs all scheduled tasks whose interval has elapsed.
 *
 * Checks each task in the task_table. If the task's interval is zero, it runs
 * every time this function is called. Otherwise, the task runs only if the
 * elapsed time since its last run is greater than or equal to its interval.
 * Updates the task's last_run_ms timestamp after running.
 */
void tasksRun(void);

#endif  // TASKS_H_
