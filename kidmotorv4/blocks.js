Blockly.Blocks['kidmotor_motor_forward'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_motor_forward",
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_FORWARD_MESSAGE,
			"args0": [{
				"type": "field_number",
				"name": "speed",
				"value": 50,
				"min": 0,
				"max": 100
			}, {
				"type": "field_number",
				"name": "time",
				"value": 1,
				"min": 0
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_FORWARD_TOOLTIP,
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['kidmotor_motor_backward'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_motor_backward",
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_BACKWARD_MESSAGE,
			"args0": [{
				"type": "field_number",
				"name": "speed",
				"value": 50,
				"min": 0,
				"max": 100
			}, {
				"type": "field_number",
				"name": "time",
				"value": 1,
				"min": 0
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_BACKWARD_TOOLTIP,
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['kidmotor_motor_turn_left'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_motor_turn_left",
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_TURN_LEFT_MESSAGE,
			"args0": [{
				"type": "field_number",
				"name": "speed",
				"value": 50,
				"min": 0,
				"max": 100
			}, {
				"type": "field_number",
				"name": "time",
				"value": 1,
				"min": 0
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_TURN_LEFT_TOOLTIP,
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['kidmotor_motor_turn_right'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_motor_turn_right",
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_TURN_RIGHT_MESSAGE,
			"args0": [{
				"type": "field_number",
				"name": "speed",
				"value": 50,
				"min": 0,
				"max": 100
			}, {
				"type": "field_number",
				"name": "time",
				"value": 1,
				"min": 0
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_TURN_RIGHT_TOOLTIP,
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['kidmotor_motor_move'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_motor_move",
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_MOVE_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "move",
				"options": [
					[ Blockly.Msg.KIDMOTOR_MOTOR_MOVE_FORWARD_MESSAGE, "1" ],
					[ Blockly.Msg.KIDMOTOR_MOTOR_MOVE_BACKWARD_MESSAGE, "2" ],
					[ Blockly.Msg.KIDMOTOR_MOTOR_MOVE_TURN_LEFT_MESSAGE, "3" ],
					[ Blockly.Msg.KIDMOTOR_MOTOR_MOVE_TURN_RIGHT_MESSAGE, "4" ]
				]
			}, {
				"type": "field_number",
				"name": "speed",
				"value": 50,
				"min": 0,
				"max": 100
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_MOVE_TOOLTIP,
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['kidmotor_motor_wheel'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_motor_wheel",
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_WHEEL_MESSAGE,
			"args0": [{
				"type": "field_number",
				"name": "speed1",
				"value": 50,
				"min": -100,
				"max": 100
			}, {
				"type": "field_number",
				"name": "speed2",
				"value": 50,
				"min": -100,
				"max": 100
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_WHEEL_TOOLTIP,
			"helpUrl": ""
		});
	}
};

Blockly.Blocks['kidmotor_motor_stop'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_motor_stop",
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_STOP_MESSAGE,
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_STOP_TOOLTIP,
			"helpUrl": ""
		});
	}
};

Blockly.Blocks["kidmotor_motor"] = {
	init: function() {
		this.jsonInit({
			"message0": Blockly.Msg.KIDMOTOR_MOTOR_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "n",
				"options": [
					[ "1", "1" ],
					[ "2", "2" ]
				]
			}, {
				"type": "field_dropdown",
				"name": "dir",
				"options": [
					[ Blockly.Msg.KIDMOTOR_MOTOR_FORWARD1_MESSAGE, "1" ],
					[ Blockly.Msg.KIDMOTOR_MOTOR_BACKWARD1_MESSAGE, "0" ]
				]
			}, {
				"type": "input_value",
				"name": "value",
				"check": "Number"
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_MOTOR_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		});
	},
	xmlToolbox: function() {
		return `
			<block type="kidmotor_motor">
				<value name="value">
					<shadow type="math_number">
						<field name="VALUE">100</field>
					</shadow>
				</value>
			</block>
		`;
	}
};

Blockly.Blocks['kidmotor_servo_set_angle'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_servo_set_angle",
			"message0": Blockly.Msg.KIDMOTOR_SERVO_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "pin",
				"options": [
					[ "SV1", "1" ],
					[ "SV2", "2" ],
					[ "SV3", "3" ],
				]
			}, {
				"type": "input_value",
				"name": "value",
				"check": [
					"Number",
					"Boolean"
				]
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_SERVO_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		});
	},
	xmlToolbox: function() {
		return `
			<block type="kidmotor_servo_set_angle">
				<value name="value">
					<shadow type="math_number">
						<field name="VALUE">90</field>
					</shadow>
				</value>
			</block>
		`;
	}
};

Blockly.Blocks['kidmotor_servo_unlock'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_servo_unlock",
			"message0": Blockly.Msg.KIDMOTOR_SERVO_UNLOCK_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "pin",
				"options": [
					[ "SV1", "1" ],
					[ "SV2", "2" ],
					[ "SV3", "3" ],
				]
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_SERVO_UNLOCK_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		});
	}
};

Blockly.Blocks['kidmotor_digital_write'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_digital_write",
			"message0": Blockly.Msg.KIDMOTOR_DIGITAL_WRITE_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "pin",
				"options": [
					[ "D1", "1" ],
					[ "D2", "2" ],
					[ "D3", "3" ],
					[ "D4", "4" ],
					[ "D5", "5" ],
				]
			}, {
				"type": "input_value",
				"name": "value",
				"check": [
					"Number",
					"Boolean"
				]
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_DIGITAL_WRITE_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		});
	},
	xmlToolbox: function() {
		return `
			<block type="kidmotor_digital_write">
				<value name="value">
					<shadow type="math_number">
						<field name="VALUE">1</field>
					</shadow>
				</value>
			</block>
		`;
	}
};

Blockly.Blocks['kidmotor_digital_read'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_digital_read",
			"message0": Blockly.Msg.KIDMOTOR_DIGITAL_READ_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "pin",
				"options": [
					[ "D1", "1" ],
					[ "D2", "2" ],
					[ "D3", "3" ],
					[ "D4", "4" ],
					[ "D5", "5" ],
				]
			}],
			"output": [
			  "Number",
			  "Boolean"
			],
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_DIGITAL_READ_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		});
	}
};

Blockly.Blocks['kidmotor_analog_read'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_analog_read",
			"message0": Blockly.Msg.KIDMOTOR_ANALOG_READ_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "pin",
				"options": [
					[ "A0", "4" ],
					[ "A1", "5" ],
				]
			}],
			"output": [
				"Number"
			],
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_ANALOG_READ_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		  });
	}
};

Blockly.Blocks['kidmotor_pwm_write'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_pwm_write",
			"message0": Blockly.Msg.KIDMOTOR_PWM_WRITE_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "pin",
				"options": [
					[ "D1", "1" ],
					[ "D2", "2" ],
					[ "D3", "3" ],
					[ "D4", "4" ],
					[ "D5", "5" ],
				]
			}, {
				"type": "input_value",
				"name": "value",
				"check": [
					"Number"
				]
			}],
			"inputsInline": true,
			"previousStatement": null,
			"nextStatement": null,
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_PWM_WRITE_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		});
	},
	xmlToolbox: function() {
		return `
			<block type="kidmotor_pwm_write">
				<value name="value">
					<shadow type="math_number">
						<field name="VALUE">1</field>
					</shadow>
				</value>
			</block>
		`;
	}
};

Blockly.Blocks['kidmotor_get_distance'] = {
	init: function() {
		this.jsonInit({
			"type": "kidmotor_get_distance",
			"message0": Blockly.Msg.KIDMOTOR_DISTANCE_MESSAGE,
			"args0": [{
				"type": "field_dropdown",
				"name": "pin_trig",
				"options": [
					[ "D1", "1" ],
					[ "D2", "2" ],
					[ "D3", "3" ],
					[ "D4", "4" ],
					[ "D5", "5" ],
				]
			},
			{
				"type": "field_dropdown",
				"name": "pin_echo",
				"options": [
					[ "D1", "1" ],
					[ "D2", "2" ],
					[ "D3", "3" ],
					[ "D4", "4" ],
					[ "D5", "5" ],
				]
			}],
			"output": [
			  "Number",
			  "Boolean"
			],
			"colour": 45,
			"tooltip": Blockly.Msg.KIDMOTOR_DISTANCE_TOOLTIP,
			"helpUrl": "https://www.ioxhop.com/p/851"
		});
	}
};
