// Minimal stand-ins for the firmware headers used by motor/boost_charge.c
#ifndef TEST_STUBS_H_
#define TEST_STUBS_H_

#include <stdbool.h>
#include <stdint.h>

#define HW_HAS_3_SHUNTS
#define VIN_R1					300000.0
#define VIN_R2					5100.0
#define ADC_VOLTS_PH_FACTOR		1.0
#define PH_DIV					((VIN_R1 + VIN_R2) / VIN_R2)

// Simulated hardware, owned by test.c
extern float sim_v_phase[3];
extern float sim_ph_gain[3];	// Divider error of each phase
extern float sim_v_bat;

#define ADC_V_L1_VOLTS			(sim_v_phase[0] * sim_ph_gain[0] / PH_DIV)
#define ADC_V_L2_VOLTS			(sim_v_phase[1] * sim_ph_gain[1] / PH_DIV)
#define ADC_V_L3_VOLTS			(sim_v_phase[2] * sim_ph_gain[2] / PH_DIV)
#define GET_INPUT_VOLTAGE()		(sim_v_bat)

#define MCCONF_FOC_OFFSETS_CURRENT_0	2048.0
#define MCCONF_FOC_OFFSETS_CURRENT_1	2048.0
#define MCCONF_FOC_OFFSETS_CURRENT_2	2048.0

// Lisp eeprom marker and stored script of the boost charge package
extern int sim_marker;
typedef union { int32_t as_i32; float as_float; } eeprom_var;
#define CODE_IND_LISP			1
static inline uint32_t flash_helper_code_size(int ind) { (void)ind; return 100; }
static inline bool conf_general_read_eeprom_var_custom(eeprom_var *v, int address) {
	(void)address; v->as_i32 = sim_marker; return true;
}

typedef enum { MOTOR_TYPE_BLDC = 0, MOTOR_TYPE_DC, MOTOR_TYPE_FOC } mc_motor_type;
typedef enum { MC_STATE_OFF = 0, MC_STATE_RUNNING } mc_state;
typedef enum { FAULT_CODE_NONE = 0, FAULT_CODE_OVER_VOLTAGE } mc_fault_code;

typedef struct {
	mc_motor_type motor_type;
	float foc_offsets_current[3];
	float foc_current_kp;
	float foc_current_ki;
	float foc_motor_l;
} mc_configuration;

const volatile mc_configuration* mc_interface_get_configuration(void);
mc_state mc_interface_get_state(void);
mc_fault_code mc_interface_get_fault(void);
float mc_interface_get_rpm(void);

static inline void chThdSleepMilliseconds(int ms) { (void)ms; }
static inline void commands_printf(const char *fmt, ...) { (void)fmt; }
static inline void terminal_register_command_callback(const char *c, const char *h, const char *a,
		void (*cbf)(int argc, const char **argv)) { (void)c; (void)h; (void)a; (void)cbf; }

static inline void utils_truncate_number(float *number, float min, float max) {
	if (*number > max) { *number = max; } else if (*number < min) { *number = min; }
}

static inline void utils_step_towards(float *value, float goal, float step) {
	if (*value < goal) {
		*value = (*value + step) < goal ? *value + step : goal;
	} else if (*value > goal) {
		*value = (*value - step) > goal ? *value - step : goal;
	}
}

#endif
