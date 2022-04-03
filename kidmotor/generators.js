const KIDMOTOR_BEGIN = `DEV_I2C1.KidMotorV3(0, 0x08)`;
const KIDMOTOR_GEN_SET_MOTOR = (ch, dir, speed) => `${KIDMOTOR_BEGIN}.setMotor(${ch}, ${dir}, ${speed});`;
const KIDMOTOR_GEN_DELAY = (t) => `vTaskDelay((${t} * 1000) / portTICK_RATE_MS);`;
const KIDMOTOR_GEN_SET_MOTOR_STOP = () => `${KIDMOTOR_GEN_SET_MOTOR(1, 0, 0)} ${KIDMOTOR_GEN_SET_MOTOR(2, 0, 0)}`;

Blockly.JavaScript['kidmotor_motor'] = function(block) {
  var dropdown_n = block.getFieldValue('n');
  var dropdown_dir = block.getFieldValue('dir');
  var value_value = Blockly.JavaScript.valueToCode(block, 'value', Blockly.JavaScript.ORDER_ATOMIC);

  var code = `${KIDMOTOR_BEGIN}.setMotor(${dropdown_n}, ${dropdown_dir}, ${value_value});\n`;
  return code;
};

Blockly.JavaScript['kidmotor_digital_read'] = function(block) {
  var dropdown_pin = block.getFieldValue('pin');
  
	var code = `${KIDMOTOR_BEGIN}.getInput(${dropdown_pin})`;
	return [code, Blockly.JavaScript.ORDER_NONE];
};

Blockly.JavaScript['kidmotor_digital_write'] = function(block) {
	var dropdown_pin = block.getFieldValue('pin');
  var value_value = Blockly.JavaScript.valueToCode(block, 'value', Blockly.JavaScript.ORDER_ATOMIC) || "0";
  
	var code = `${KIDMOTOR_BEGIN}.setOutput(${dropdown_pin}, ${value_value});\n`;
	return code;
};

Blockly.JavaScript['kidmotor_analog_read'] = function(block) {
  var dropdown_pin = block.getFieldValue('pin');
  
	var code = `${KIDMOTOR_BEGIN}.getADC(${dropdown_pin})`;
	return [code, Blockly.JavaScript.ORDER_NONE];
};

Blockly.JavaScript['kidmotor_motor_forward'] = function(block) {
  var number_speed = block.getFieldValue('speed');
  var number_time = block.getFieldValue('time');
  
  var code = `${KIDMOTOR_GEN_SET_MOTOR(1, 1, number_speed)} ${KIDMOTOR_GEN_SET_MOTOR(2, 1, number_speed)} ${KIDMOTOR_GEN_DELAY(number_time)} ${KIDMOTOR_GEN_SET_MOTOR_STOP()}\n`;
  return code;
};

Blockly.JavaScript['kidmotor_motor_backward'] = function(block) {
  var number_speed = block.getFieldValue('speed');
  var number_time = block.getFieldValue('time');

  var code = `${KIDMOTOR_GEN_SET_MOTOR(1, 0, number_speed)} ${KIDMOTOR_GEN_SET_MOTOR(2, 0, number_speed)} ${KIDMOTOR_GEN_DELAY(number_time)} ${KIDMOTOR_GEN_SET_MOTOR_STOP()}\n`;
  return code;
};

Blockly.JavaScript['kidmotor_motor_turn_left'] = function(block) {
  var number_speed = block.getFieldValue('speed');
  var number_time = block.getFieldValue('time');
  var code = `${KIDMOTOR_GEN_SET_MOTOR(1, 1, 0)} ${KIDMOTOR_GEN_SET_MOTOR(2, 1, number_speed)} ${KIDMOTOR_GEN_DELAY(number_time)} ${KIDMOTOR_GEN_SET_MOTOR_STOP()}\n`;
  return code;
};

Blockly.JavaScript['kidmotor_motor_turn_right'] = function(block) {
  var number_speed = block.getFieldValue('speed');
  var number_time = block.getFieldValue('time');
  var code = `${KIDMOTOR_GEN_SET_MOTOR(1, 1, number_speed)} ${KIDMOTOR_GEN_SET_MOTOR(2, 0, 0)} ${KIDMOTOR_GEN_DELAY(number_time)} ${KIDMOTOR_GEN_SET_MOTOR_STOP()}\n`;
  return code;
};

Blockly.JavaScript['kidmotor_motor_move'] = function(block) {
  var dropdown_move = block.getFieldValue('move');
  var number_speed = block.getFieldValue('speed');
  var code = '';;
  if (dropdown_move == 1) {
    code = `${KIDMOTOR_GEN_SET_MOTOR(1, 1, number_speed)} ${KIDMOTOR_GEN_SET_MOTOR(2, 1, number_speed)}\n`;
  } else if (dropdown_move == 2) {
    code = `${KIDMOTOR_GEN_SET_MOTOR(1, 0, number_speed)} ${KIDMOTOR_GEN_SET_MOTOR(2, 0, number_speed)}\n`;
  } else if (dropdown_move == 3) {
    code = `${KIDMOTOR_GEN_SET_MOTOR(1, 1, 0)} ${KIDMOTOR_GEN_SET_MOTOR(2, 1, number_speed)}\n`;
  } else if (dropdown_move == 4) {
    code = `${KIDMOTOR_GEN_SET_MOTOR(1, 1, number_speed)} ${KIDMOTOR_GEN_SET_MOTOR(2, 0, 0)}\n`;
  }
  return code;
};

Blockly.JavaScript['kidmotor_motor_wheel'] = function(block) {
  var number_speed1 = block.getFieldValue('speed1');
  var number_speed2 = block.getFieldValue('speed2');

  var code = `${KIDMOTOR_GEN_SET_MOTOR(1, `${number_speed1} > 0 ? 1 : 0`, `${number_speed1} > 0 ? ${number_speed1} : (${number_speed1} * -1)`)} ${KIDMOTOR_GEN_SET_MOTOR(2, `${number_speed2} > 0 ? 1 : 0`, `${number_speed2} > 0 ? ${number_speed2} : (${number_speed2} * -1)`)}\n`;
  return code;
};

Blockly.JavaScript['kidmotor_motor_stop'] = function(block) {
  var code = `${KIDMOTOR_GEN_SET_MOTOR_STOP()}\n`;
  return code;
};
