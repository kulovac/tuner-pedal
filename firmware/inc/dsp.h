#include <stddef.h>
#include <stdint.h>
#define BUFFER_SIZE 1024

/**
 *  @brief computes the base frequency
 *  @param sig Audio signal
 */
float compute_yin(float sig[BUFFER_SIZE]);

/**
 * @brief initialize data structures for dsp calcs
 * @note must be run before @ref compute_yin
 */
void init_dsp(void);

/**
 * @brief gets the number of cents the signal is off
 * from the nearest note
 * @param freq The signal frequency in Hz
 */
float cents_diff(float freq);

/**
 * @brief gets the nearest note in a string format
 * @param freq The signal frequency in Hz
 */
char *get_note(float freq);
