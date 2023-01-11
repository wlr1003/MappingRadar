/*
 * lsm6dsl.c
 *
 *  Created on: Jan 9, 2023
 *      Author: William
 */

/*
 * Initialization
 */
uint8_t LSM6DSL_Initialize(LSM6DSL *ptr) {
	return 0;
}

/*
 * Data Acquisition
 */
HAL_StatusTypeDef LSM6DSL_ReadTemperature(LSM6DSL *ptr) {
	return 0;
}
HAL_StatusTypeDef LSM6DSL_ReadAcceleration(LSM6DSL *ptr) {
	return 0;
}
HAL_StatusTypeDef LSM6DSL_ReadGyroscope(LSM6DSL *ptr) {
	return 0;
}

/*
 * Low Level Functions
 */

HAL_StatusTypeDef LSM6DSL_ReadRegister(LSM6DSL *ptr, uint8_t reg, uint8_t *data) {
	return 0;
}

HAL_StatusTypeDef LSM6DSL_ReadRegisters(LSM6DSL *ptr, uint8_t reg, uint8_t *data, uint8_t length) {
	return 0;
}

HAL_StatusTypeDef LSM6DSL_WriteRegister(LSM6DSL *ptr, uint8_t reg, uint8_t *data) {
	return 0;
}
