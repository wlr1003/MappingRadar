/*
 * lsm6dsl.c
 *
 *  Created on: Jan 9, 2023
 *      Author: William
 */

// Flag to indicate when the SPI transfer is complete
volatile uint8_t spi_complete_flag = 0;

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
	uint8_t tx_data[2];
	tx_data[0] = 0x80 | reg;
	tx_data[1] = 0x00;

	// Reset the complete flag
	spi_complete_flag = 0;

	// Start the SPI transfer
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive_DMA(&hspi1, tx_data, data, 2);

	// Wait for the transfer to complete
	while(!spi_complete_flag);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return status;
}

HAL_StatusTypeDef LSM6DSL_ReadRegisters(LSM6DSL *ptr, uint8_t reg, uint8_t *data, uint8_t length) {
	return 0;
}

HAL_StatusTypeDef LSM6DSL_WriteRegister(LSM6DSL *ptr, uint8_t reg, uint8_t *data) {
	{
	    uint8_t tx_data[2];
	    tx_data[0] = reg;
	    tx_data[1] = data;

	    // Reset the complete flag
	    spi_complete_flag = 0;

	    // Start the SPI transfer
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	    status=HAL_SPI_Transmit_DMA(&hspi1, tx_data, 2);

	    // Wait for the transfer to complete
	    while(!spi_complete_flag);

	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	    return status;
	}

	// Interrupt callback function for the SPI DMA transfer complete interrupt
	void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
	{
	    spi_complete_flag = 1;
	}
