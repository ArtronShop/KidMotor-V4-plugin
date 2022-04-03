#include "kidmotorv3.h"

/*
  DEV By IOXhop : www.ioxhop.com
  Sonthaya Nongnuch : fb.me/maxthai
*/

KidMotorV3::KidMotorV3(int bus_ch, int dev_addr) {
	channel = bus_ch;
	address = dev_addr;
	polling_ms = 100;
}

void KidMotorV3::init(void) {
	// set initialized flag
	this->initialized = true;
	
	// clear error flag
	this->error = false;
	
	// Clear
	memset(KidMotorData, 0, 2);
}

int KidMotorV3::prop_count(void) {
	// not supported
	return 0;
}

bool KidMotorV3::prop_name(int index, char *name) {
	// not supported
	return false;
}

bool KidMotorV3::prop_unit(int index, char *unit) {
	// not supported
	return false;
}

bool KidMotorV3::prop_attr(int index, char *attr) {
	// not supported
	return false;
}

bool KidMotorV3::prop_read(int index, char *value) {
	// not supported
	return false;
}

bool KidMotorV3::prop_write(int index, char *value) {
	// not supported
	return false;
}
// --------------------------------------

// Start here
void KidMotorV3::process(Driver *drv) {
	i2c = (I2CDev *)drv;
}

// Method
void KidMotorV3::setMotor(uint8_t ch, uint8_t dir, uint8_t speed) {
	if (speed > 100) speed = 100;
	speed = speed * 80.0 / 100.0; // Fixed Bat cut

	uint8_t data[2] = {
		(uint8_t)((ch - 1) == 0 ? 0x00 : 0x01), 
		(uint8_t)((dir << 7) | (speed & 0x7F))
	};
	
	this->error = this->i2c->write(this->channel, this->address, data, 2) ==  ESP_OK;
}

void KidMotorV3::setMode(uint8_t ch, uint8_t mode) {
	ch = ch - 1;

	static uint8_t mode_tmp = 0;
	if (mode == 1) {
		mode_tmp |= 1 << ch;
	} else {
		mode_tmp &= ~(1 << ch);
	}

	uint8_t data[] = {
		0x02, 
		mode_tmp
	};

	this->error = this->i2c->write(this->channel, this->address, data, 2) ==  ESP_OK;
}

void KidMotorV3::setOutput(uint8_t ch, bool val) {
	this->setMode(ch, MODE_OUTPUT);

	ch = ch - 1;
	static uint8_t out_tmp = 0;
	if (val) {
		out_tmp |= 1 << ch;
	} else {
		out_tmp &= ~(1 << ch);
	}
	
	uint8_t data[] = {
		0x04, 
		out_tmp
	};

	this->error = this->i2c->write(this->channel, this->address, data, 2) ==  ESP_OK;
}

int KidMotorV3::getInput(uint8_t ch) {
	this->setMode(ch, MODE_INPUT);

	ch = ch - 1;
	
	uint8_t reg = 0x03;
	uint8_t dataIn = 0;

	this->error = this->i2c->read(this->channel, this->address, &reg, 1, &dataIn, 1) ==  ESP_OK;

	return (dataIn & (1 << ch)) ? 1 : 0;
}

int KidMotorV3::getADC(uint8_t ch) {
	ch = ch - 1;
	uint8_t data[2];
	uint8_t buff[2];

	data[0] = 0x05;
	data[1] = 0x80 | (ch & 0x07); // Write ADC ch and set FLAG

	this->error = this->i2c->write(this->channel, this->address, data, 2) ==  ESP_OK;

	uint8_t try_count = 0;
	while(try_count < 100) {
		data[0] = 0x05;
		this->error = this->i2c->read(this->channel, this->address, &data[0], 1, &buff[0], 1);
		if ((buff[0] & 0x80) == 0) {
			break;
		} else {
			vTaskDelay(1 / portTICK_RATE_MS);
			try_count++;
		}
	}

	if (try_count >= 100) {
		return 0;
	}

	data[0] = 0x06;
	memset(buff, 0, 2);
	this->error = this->i2c->read(this->channel, this->address, &data[0], 1, buff, 2);

	return (buff[0] << 8) | buff[1];
}
