// -----------------------------------------------------------------------------
// Author       :   Luis de la Barba & Javier R. Juliani
// Date         :   17/04/2020
// Adaptation   :   Juan Mata & Jaime Landa
// Date         :   03/2024
// Name         :   module_state_machine.c
// Description  :
// * This file is for the finite states machine
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "module_state_machine.h"

// INITIALIZE VARIABLES
BMS_MOD BMS[] = {
//Here all the BMS get declare, the directions come from Zeva and the LAG of 50, 100 etc is for not asking them voltages at once
		BMS_MOD(BMS_ID + 00, BMS_MAXV, BMS_MINV, BMS_MAXT, 10, BMS_SHUNT), // 3+3+3+3
		BMS_MOD(BMS_ID + 10, BMS_MAXV, BMS_MINV, BMS_MAXT, 9, BMS_SHUNT, 50), // 3+5
		BMS_MOD(BMS_ID + 20, BMS_MAXV, BMS_MINV, BMS_MAXT, 10, BMS_SHUNT, 100), // 5+5
		BMS_MOD(BMS_ID + 30, BMS_MAXV, BMS_MINV, BMS_MAXT, 9, BMS_SHUNT, 150), // 5+5
		BMS_MOD(BMS_ID + 40, BMS_MAXV, BMS_MINV, BMS_MAXT, 10, BMS_SHUNT, 200), // 5+5
		BMS_MOD(BMS_ID + 50, BMS_MAXV, BMS_MINV, BMS_MAXT, 9, BMS_SHUNT, 250), // 5+5
		BMS_MOD(BMS_ID + 60, BMS_MAXV, BMS_MINV, BMS_MAXT, 10, BMS_SHUNT, 300), // 5+5
		BMS_MOD(BMS_ID + 70, BMS_MAXV, BMS_MINV, BMS_MAXT, 9, BMS_SHUNT, 350), // 5+5
		BMS_MOD(BMS_ID + 80, BMS_MAXV, BMS_MINV, BMS_MAXT, 9, BMS_SHUNT, 400), // 5+5
		BMS_MOD(BMS_ID + 90, BMS_MAXV, BMS_MINV, BMS_MAXT, 10, BMS_SHUNT, 450), // 5+5
		};

int BMS_N = 10;
int MIN_V = 4200;
uint8_t message_MINV[2] = { 0, 0 }; //Here I'll get the minimun voltages for sending them for telemetry
int time_sending_minV = 0;      //For checking the interval I send the messages
uint8_t message_MAXT[2] = { 0, 0 };
int time_sending_maxT = 0;

Temperatures_MOD Tempt[] = {
//Inicializo los modulos de temperatura. Cada modulo tiene un ID pero para pedir temperaturas uso el id 0x500
		Temperatures_MOD(0x510, T_MAX, 55),  // MODULE 1
		Temperatures_MOD(0x520, T_MAX, 155), // MODULE 2
		Temperatures_MOD(0x530, T_MAX, 255), // MODULE 3
		Temperatures_MOD(0x540, T_MAX, 355), // MODULE 4
		Temperatures_MOD(0x550, T_MAX, 455), // MODULE 5

		};
int Temp_N = 5;

CPU_MOD CPU(CPU_ID_send, CPU_ID_recv, 500); //Same with CPU, rest of vehicle

int flag_charger = 0; //For knowing wether I am charging or in the car

// SEV_MOD SEVCON(SEV_ID,SEV_ID+1,SEV_ID+2,SEV_ID+3,SEV_ID+4); //Do not worry about anything of these
// CHARGER_MOD CHARGER(CHARGER_ID_SEND, CHARGER_ID_RECV, CHARGER_MAXV, CHARGER_MAXI, CHARGER_MINI);

Current_MOD current(Current_ID, Current_max); //Class for current measurement

static STATE state = start; //Variable on the state machine

char buffer[80];

/*********************************************************************************************************
 ** Function name:           setup_state_machine
 ** Descriptions:            Initializes the state machine library
 *********************************************************************************************************/
void setup_state_machine() {
	HAL_GPIO_WritePin(AMS_OK_GPIO_Port, AMS_OK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RELAY_AIR_N_GPIO_Port, RELAY_AIR_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RELAY_AIR_P_GPIO_Port, RELAY_AIR_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin,
			GPIO_PIN_RESET);
}

/*********************************************************************************************************
 ** Function name:           select_state
 ** Descriptions:            check what should they do on the state machine
 *********************************************************************************************************/
void select_state() {
	int state_air_n = 0;     // 0 means open, 1 closed     // Turn on SEVCON
	int state_air_p = 0;     // 0 means open, 1 closed     // Energize HV relay
	int state_precharge = 0; // 0 means open, 1 closed
	int flag_cpu = CPU_ERROR_COMMUNICATION;
	int flag_current = Current_ERROR_Comunication;

	/*
	 * TIM16 -> APB2 => 264MHz
	 * 1 ms interruption => 1ms * 264MHz = 264000
	 * prescalado 264 (por ejemplo)
	 * timer count = 264000 / 264 = 1000
	 */

	uint32_t time = HAL_GetTick();
	int time_s = HAL_GetTick();

	CPU.voltage_acum = 0; // For precharge

	MIN_V = 4200; /// I reset the number each cycle cause if the voltages goes up again I wanna has it risen again on telemetry
	for (int i = 0; i < BMS_N; i++) {
		BMS[i].voltage_acum = 0;                  // For precharge
		if (BMS[i].query(time, buffer) != BMS_OK) //I ask the BMS about voltages and cheking their states
		{
			state = error;
		}
		CPU.voltage_acum += BMS[i].voltage_acum; // For precharge
		if (BMS[i].MIN_V < MIN_V)
			MIN_V = BMS[i].MIN_V; //Checking the minimun voltage of cell in the whole battery
	}
	if (time_s > time_sending_minV + 500) {
		message_MINV[1] = MIN_V & 0xFF;
		message_MINV[0] = (MIN_V >> 8) & 0xFF;
		if (BMS[0].flag_charger != 1) {
			if (module_send_message_NoExtId_CAN1(BMS_ID, message_MINV, 2) != HAL_OK)
				print((char*) "Error al enviar tension minima"); //Sending the message through telemtry each 500 ms
		}
		time_sending_minV = time_s;
	}
	for (int i = 0; i < BMS_N; i++) {
		BMS[i].BALANCING_V = MIN_V; //Here I say I wanna balance all the cells in the battery to the minimun

	}
	if (time > 1000 && time < 2000)
		MIN_V = 4200; //Do not really worry about this, I dunno remember if this was really necesary
	int MAX_T = 0;
	for (int i = 0; i < Temp_N; i++) {
		if (Tempt[i].query(time, buffer) != Temperatures_OK)
			state = error; //Asking how temperatures are
		if (Tempt[i].MAX_T > MAX_T)
			MAX_T = Tempt[i].MAX_T;
	}

	if (time_s > time_sending_maxT + 1000) {
		message_MAXT[1] = MAX_T & 0xFF;
		message_MAXT[0] = 0;
		if (Tempt[0].flag_charger != 1) {
			if (module_send_message_NoExtId_CAN1(Temp_ID, message_MAXT, 2) != HAL_OK)
				print((char*) "Error al enviar temperatura maxima"); //Sending the message through telemetry each 1000 ms
		}
		time_sending_maxT = time_s;

	}

	flag_cpu = CPU.query(time, buffer); //Asking the rest of the car how is it

	//flag_current = current.query(time, buffer); //asking current how is it

	switch (state) {
	case start:
		state_air_n = 0;
		state_air_p = 0;
		state_precharge = 0;
		CPU.updateState(CPU_DISCONNECTED);
		if (flag_cpu != CPU_ERROR_COMMUNICATION)
			state = precharge; //If I do comunicate with the rest of the car, I go to precharge
		break;
	case precharge:
		state_air_n = 1;
		state_air_p = 0;
		state_precharge = 1;
		CPU.updateState(CPU_PRECHARGE);
		if (flag_cpu == CPU_OK) {
			state = transition;
		} else if (flag_cpu == CPU_ERROR_COMMUNICATION)
			state = error;
		// else if(flag_current != Current_OK) state = error; //I take this out cause in precharge current can be very high, but probably can be uncommented,
		break;
	case transition:
		state_air_n = 1;
		state_air_p = 0;
		state_precharge = 1;
		CPU.updateState(CPU_PRECHARGE);
		if (CPU.voltage_acum * 0.9 < CPU.DC_BUS)
			state = run; //If DC_BUS voltage is higher than 90% of battery voltage, precharge finish
		// else if((flag_cpu == CPU_ERROR_COMMUNICATION)&&(flag_charger == 1)) state = error;
		// else if(flag_current != Current_OK) state = error;
		break;
	case run:
		state_air_n = 1;
		state_air_p = 1;
		state_precharge = 1;
		CPU.updateState(CPU_POWER);
		if ((flag_cpu == CPU_ERROR_COMMUNICATION) && (flag_charger == 1))
			state = error; //If I disconnect the charger, error
		//if (flag_current != Current_OK)
			//state = error; //If current is too high, error
		break;
	case error:
		state_air_n = 0; //All relés closed
		state_air_p = 0;
		state_precharge = 0;
		CPU.updateState(CPU_ERROR);
		break;
	}
	/*   Serial.print("State: ");
	 Serial.println(state);
	 Serial.print("State AIR+: ");
	 Serial.println(state_air_n);
	 Serial.print("Relee Prec:");
	 Serial.println(state_precharge); */

	/*sprintf(buffer, "\n***********************\n");
	 print(buffer);
	 sprintf(buffer, " - STATE:     %i\n", state);
	 print(buffer);
	 sprintf(buffer, "***********************\n");
	 print(buffer);*/
	HAL_GPIO_WritePin(AMS_OK_GPIO_Port, AMS_OK_Pin,
			state_air_n ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RELAY_AIR_N_GPIO_Port, RELAY_AIR_N_Pin,
			state_air_n ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RELAY_AIR_P_GPIO_Port, RELAY_AIR_P_Pin,
			state_air_p ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RELAY_PRECHARGE_GPIO_Port, RELAY_PRECHARGE_Pin,
			state_precharge ? GPIO_PIN_SET : GPIO_PIN_RESET);

	printnl((char*)"State: ");
	printValue(state);
	printnl((char*)"State AIR+: ");
	printValue(state_air_p);
	printnl((char*)"State AIR-: ");
	printValue(state_air_n);
	printnl((char*)"Relee Prec:");
	printValue(state_precharge);


}

/*********************************************************************************************************
 ** Function name:           parse_state
 ** Descriptions:            Function for analysing the data from the CAN
 *********************************************************************************************************/
void parse_state(CANMsg data) {
	uint32_t time = HAL_GetTick();
	bool flag_bms = false;
	bool flag_temperatures = false;

	for (int i = 0; i < BMS_N; i++) {
		flag_bms = BMS[i].parse(data.id, &data.buf[0], time); //Checking if the message received is for  BMS
		if (flag_bms)
			i = BMS_N;
	}

	for (int i = 0; i < Temp_N; i++) {
		flag_temperatures = Tempt[i].parse(data.id, &data.buf[0], time); //Checking if the message received is for temperatures

		if (flag_temperatures)
			i = Temp_N;
	}
	if (!flag_bms && !flag_temperatures) {
		if (CPU.parse(data.id, &data.buf[0], time))
			;                       //Cheking if message is for CPU
		if (data.id == 419385575) //If message from this direction received, it is because the charger is connected and the accu is for charging
				{
			for (int i = 0; i < BMS_N; i++) {
				BMS[i].flag_charger = 1;
			}
			for (int i = 0; i < Temp_N; i++) {
				Tempt[i].flag_charger = 1;
			}
			current.flag_charger = 1;
			flag_charger = 1;
			// Serial.print("Charger: ");
			// Serial.println(flag_charger);
		} else {
			// Serial.print("ID: ");
			// Serial.println(data.id);
		}
	}
}

