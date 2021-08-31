#pragma once
#include <stdio.h>
#include "Table.h"
#include "Vec3.h"
#include "State.h"
#include "Input.h"
#include "AeroData_1.h"
#include "Engine.h"
#include "Airframe.h"
#include "Fuel_System.h"
#include "BaseComponent.h"
#include "Timer.h"

//=========================================================================//
//
//		FILE NAME	: FlightModel.cpp
//		AUTHOR		: Joshua Nelson & Paul Stich
//		Copyright	: Joshua Nelson & Paul Stich
//		DATE		: August 2021
//
//		DESCRIPTION	: Every force that is needed for the flight-simulation. 
//					  As well as the Camera-Shaker.
//					  
//================================ Includes ===============================//
//=========================================================================//

class FlightModel
{
public:
	FlightModel(State& state, Input& input, Engine& engine, Airframe& airframe); //Engine& engine NEU 21FEb21

	virtual void zeroInit();
	virtual void coldInit();
	virtual void hotInit();
	virtual void airborneInit();
	
	void update(double dt);


	void calculateAero();
	void thrustForce();

	//----------Cockpit-Shaker--------------------------
	void calculateShake(double& dt);
	inline double getCockpitShake();
	//inline void setCockpitShakeModifier(double mod);


	inline const Vec3& getForce() const; //inline u.U. unnötig, da die Funktion nicht direkt hier implementiert wurde; inline könnte weg
	inline const Vec3& getMoment() const; //inline u.U. unnötig, da die Funktion nicht direkt hier implementiert wurde; inline könnte weg

private:
	Vec3 m_moment;
	Vec3 m_force;
	State& m_state;
	Input& m_input;
	Engine& m_engine; //neu 21Feb21 // wieder rausgenommen
	Airframe& m_airframe;

	//--------------Aerodynamic Values--------------------------------
	double m_scalarVelocity = 0.0;
	double m_scalarVelocitySquared = 0.0;
	double m_aoaDot = 0.0;
	double m_aoaPrevious = 0.0;
	//--------------Thrust related Values-----------------------------
	//double m_thrust = 0.0; //neu 21FEb21 rauskommentiert 28.02.2021
	//--------------Misc---------------------------------------------
	//--------------PitchUp and Stall--------------------------------
	double m_pitchup = 0.0;
	double m_stallMult = 0.0;

	//--------------Formula-Parts STAB--------------------------------
	//s : area
	//b : wingspan
	//p : density
	//V : scalar velocity
	double m_k = 0.0; //0.5*p*V^2 * s
	double m_q = 0.0; //0.5*p*V^2 * s * b
	double m_p = 0.0; //0.25*p*V * s * b^2

	//-----------------Formula-Parts DRAG----------------------------
	double CDwave = 0.0; // a * (M/Mcrit)^b // neu 18.02.2021
	double M_mcrit = 0.0; // Mach/Mcrit // neu 18.02.2021
	double M_mcrit_b = 0.0; //(Mach/Mcrit)^CON_wdb // neu 18.02.2021
	double CDi = 0.0; //(CL^2/pi * AR * e) neu 18.02.2021
	double CDGear = 0.0; //NEU 05April21 Gear-Down-Drag
	double CDFlaps = 0.0; //NEU 05April21 Flaps-Down-Drag
	double CLFlaps = 0.0;//NEU 05April21 Flaps-Down-Lift
	double CDBrk = 0.0;// NEU 05April21 Brk-Out-Drag
	double CDBrkCht = 0.0;//NEU und Drag Brake-Chute
	double CLblc = 0.0; //NEU BLC Lift-System für die angeströmten Landeklappen

	//---------------damage varaiables for parts---------------------
	double m_rWingDamageCL = 0.0;
	double m_rWingDamageCD = 0.0;
	double m_lWingDamageCL = 0.0;
	double m_lWingDamageCD = 0.0;
	double m_ailDamage = 0.0;
	double m_flapDamage = 0.0;
	double m_rSpdBrkDamage = 0.0;
	double m_lSpdBrkDamage = 0.0;
	double m_hStabDamage = 0.0;
	double m_vStabDamage = 0.0;

	//-------------Shaker-Stuff-------------------------------
	double m_cockpitShake = 0.0;
	double m_cockpitShakeModifier = 0.0;

	bool prevGearShake = false;
	bool gearShake = false;
	Timer m_shakeDuration;



	//--------------Tables from AeroData_1.h------------------------
	//--------------PITCH--------------------------
	Table Cmalpha;
	Table CmalphaNEW;
	Table Cmde;
	Table CmdeNEW;
	Table Cmq;
	Table Cmadot; //genau so für jede DATA_Table
	Table CmM;
	//-----------------DRAG------------------------
	Table CDmin;
	Table CDmach;
	Table CDa;
	Table CDeng;
	//-------------LIFT----------------------------
	Table CLmax;
	Table CLmach;
	Table CLa;
	Table CLds;
	//-------------Roll---------------------------
	Table Clb;
	Table Clp;
	Table Clda;
	Table Clr;
	Table Cldr;
	//--------------YAW---------------------------
	Table Cnb;
	Table Cndr;
	Table Cnr;
	Table Cyb;
	Table Cydr;
	//-------------Thrust------------------------
	//Table PMax;
	//Table PFor;
	//------------Misc---------------------------
	//------------PitchUp and Stall-------------
	Table PitAoA;
	Table PitMult;
	Table StAoA;
};

const Vec3& FlightModel::getForce() const
{
	return m_force;
}

const Vec3& FlightModel::getMoment() const
{
	return m_moment;
}

double FlightModel::getCockpitShake()
{
	return m_cockpitShake;
}

/*void FlightModel::setCockpitShakeModifier(double mod)
{
	m_cockpitShakeModifier = mod;
	m_cockpitShakeModifier = 1.5;
}*/
