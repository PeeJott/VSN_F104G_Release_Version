#include "FlightModel.h"

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


FlightModel::FlightModel
(
	State& state,
	Input& input,
	Engine& engine, //NEU 21Feb21
	Airframe& airframe
	//der letzte Eintrag hier darf kein Komma haben!
) :
	m_state(state),
	m_input(input),
	m_engine(engine), //NEU 21Feb21
	m_airframe(airframe),

	m_scalarVelocity(0.0),
	m_scalarVelocitySquared(0.0),
	m_aoaDot(0.0),
	m_aoaPrevious(0.0),

	m_k(0.0),
	m_q(0.0),
	m_p(0.0),

	m_cockpitShake(0.0),
	m_shakeDuration(0.1),

	//-----"init" Tables AeroData_1.h + FlightModel.h-----------
	//-----------------Pitch------------------------------------
	Cmalpha(DAT_Cma_SL, CON_Cma_SL_min, CON_Cma_SL_max),
	CmalphaNEW(DAT_Cma_full, CON_Cma_full_min, CON_Cma_full_max),
	Cmde(DAT_Cmde_SL, CON_Cmde_SL_min, CON_Cmde_SL_max),
	CmdeNEW(DAT_Cmde_full, CON_Cmde_full_min, CON_Cmde_full_max),
	Cmq(DAT_Cmq, CON_Cmq_min, CON_Cmq_max),
	CmadotNEW(DAT_CmaDOT_Full, CON_CmaDOT_Full_Min, CON_CmaDOT_Full_Max),
	CmM(DAT_CmM, CON_Cmq_min, CON_Cmq_max),
	CmqStAg(DAT_CmqStAg, CON_CmqStAg_Min, CON_CmqStAg_Max),
	CmaDOTStAg(DAT_CmaDOTStAg, CON_CmaDOTStAg_Min, CON_CmaDOTStAg_Max),
	//----------------DRAG---------------------------------------
	CDmin(DAT_CDmin, CON_CDminmin, CON_CDminmax),
	CDmach(DAT_CDmach_SL, CON_CDmach_SLmin, CON_CDmach_SLmax),
	CDa(DAT_CDa_SL, CON_CDa_SLmin, CON_CDa_SLmax),
	CDeng(DAT_CDeng, CON_CDengmin, CON_CDengmax),
	CDOM(DAT_CD_OM, CON_CD_OM_Min, CON_CD_OM_Max),
	CDminAD(DAT_CDminAD, CON_CDminAD_Min, CON_CDminAD_Max),
	CDOK(DAT_CD_OK, CON_CD_OK_Min, CON_CD_OK_Max),
	//---------------LIFT----------------------------------------
	CLmax(DAT_CLmax, CON_CLmaxmin, CON_CLmaxmax),
	CLmach(DAT_CLmach_SL, CON_CLmach_SLmin, CON_CLmach_SLmax),
	CLa(DAT_CLa, CON_CLamin, CON_CLamax),
	CLds(DAT_CLds, CON_CLdsmin, CON_CLdsmax),
	//CLa_FL1(DAT_CLa_FL1, CON_CLa_FL1_Min, CON_CLa_FL1_Max),
	CLa_FL2(DAT_CLa_FL2, CON_CLa_FL2_Min, CON_CLa_FL2_Max),
	CL_FlStat(DAT_CL_FlStat, CON_CL_FlStat_Min, CON_CL_FlStat_Max), //Static Flap Lift "base-lift"
	//--------------ROLL----------------------------------------
	Clb(DAT_Clb_SL, CON_Clb_SLmin, CON_Clb_SLmax),
	Clp(DAT_Clp_SL, CON_Clp_SLmin, CON_Clp_SLmax),
	Clda(DAT_Clda, CON_Cldamin, CON_Cldamax),
	Clr(DAT_Clr_SL, CON_Clr_SLmin, CON_Clr_SLmax),
	Cldr(DAT_Cldr_full, CON_Cldr_min, CON_Cldr_max),
	//---------------YAW---------------------------------------
	CnbNEW(DAT_Cnb_FULL, CON_Cnb_Full_Min, CON_Cnb_FULL_Max),
	Cndr(DAT_Cndr, CON_Cndrmin, CON_Cndrmax),
	Cnr(DAT_Cnr, CON_Cnrmin, CON_Cnrmax),
	Cyb(DAT_Cyb, CON_Cybmin, CON_Cybmax),
	Cydr(DAT_Cydr, CON_Cydrmin, CON_Cydrmax),
	
	//---------------Thrust------------------------------------
	//PMax(DAT_PMax, CON_PMaxmin, CON_PMaxmax)
	//PFor(DAT_PFor, CON_PFormin, CON_PFormax)
	ThinAM(DAT_ThinAM, CON_ThinAM_Min, CON_ThinAM_Max),
	//---------------Misc-------------------------------------
	//--------------PitchUP and Stall------------------------
	PitAoA(DAT_PitchAoA, CON_PitAoAMin, CON_PitAoAMax),
	PitMult(DAT_PitchMult, CON_PitMulMin, CON_PitMulMax),
	StAoA(DAT_StallAoA, CON_StAoAMin, CON_StAoAMax),
	StAoAMulti(DAT_StAoAMulti, CON_StAoAMulti_Min, CON_StAoAMulti_Max)
	
	//der letzte Eintrag darf KEIN Komma haben...



	//genau so für jede DATA Table (DATA, min , max)
{
	//printf("Hello\n");
}

void FlightModel::zeroInit()
{
	m_q = 0.0;
	m_p = 0.0;
	m_k = 0.0;
	CLblc = 0.0;

	m_thinAirMulti = 0.0;

	m_scalarVelocitySquared = 0.0;
	m_scalarVelocity = 0.0;
	m_aoaPrevious = 0.0;
	//m_aoaPrevious = 0.0;
	m_aoaDot = 0.0;

	m_ailDeflection = 0.0;
	m_rudDeflection = 0.0;
	
	//m_state.m_mach = 0.0;
	//m_state.m_beta = 0.0;
	//m_state.m_omega = 0.0;
	
	m_pitchup = 0.0;
	m_stallMult = 0.0;

	m_CmqStAg = 0.0;
	m_CmaDOTStAg = 0.0;
	m_StabAugSys = 0.0;

	M_mcrit = 0.0;
	M_mcrit_b = 0.0;

	CDwave = 0.0;

	CDi = 0.0;

	CD_OverMach = 0.0;

	m_cdminADD = 0.0;

	CD_brFlap = 0.0;
	m_flapBroken = false;

	CDGear = 0.0;
	CDFlaps = 0.0;
	CLFlaps = 0.0;
	CDBrk = 0.0;
	CDBrkCht = 0.0;
	CLblc = 0.0;

	m_rWingDamageCL = 0.0;
	m_rWingDamageCD = 0.0;
	m_lWingDamageCL = 0.0;
	m_lWingDamageCD = 0.0;
	m_ailDamage = 0.0;
	m_flapDamage = 0.0;
	m_rSpdBrkDamage = 0.0;
	m_rSpdBrkDamage = 0.0;
	m_hStabDamage = 0.0;
	m_vStabDamage = 0.0;
	
	
	m_moment = Vec3();
	m_force = Vec3();
	
	gearShake = false;
	prevGearShake = false;
}

void FlightModel::coldInit()
{
	zeroInit();
}

void FlightModel::hotInit()
{
	zeroInit();
}

void FlightModel::airborneInit()
{
	zeroInit();
}

void FlightModel::L_stab()
{
	//set roll moment -- 
	//Neu eingefügt am 14.02.2021 PJ-- "-" vor Clr eingefügt, da Clr Daten positiv waren und negativ sein müssten da Dämpfung
	//m_moment.x-- "2 *" vor Clda eingefügt für stärkere Ailerons = schon besser-- "2*" vor Clp eingefügt -- "0,5 *" vor Clb eingefügt
	//m_ailDamage als multiplikator eingefügt; (m_lWingDamageCD + m_rWingDamageCD) bzgl. WingDamageRoll-Effekt eingefügt;
	
	//-------------------------ALTE Version ohne Beschränkung des Querruder-Ausschlags-------------------------------------------------
	/*m_moment.x += m_q * (Clb(m_state.m_mach) * m_state.m_beta + Clda(m_state.m_mach) * ((m_input.getRoll() + m_input.getTrimmAilR() - m_input.getTrimmAilL()) * m_ailDamage ) + (m_lWingDamageCD + m_rWingDamageCD) + (0.55 * Cldr(m_state.m_mach)) * m_input.getYaw() )
		+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b * (2.0 * Clp(m_state.m_mach) * m_state.m_omega.x + (1.5 * -Clr(m_state.m_mach)) * m_state.m_omega.y);
	*/

	//-------------------------NEUE Versíon MIT Beschränkung des Max-Ausschalgs--------------------------------------------------------
	m_moment.x += m_q * (Clb(m_state.m_mach) * m_state.m_beta + Clda(m_state.m_mach) * (((m_input.getRoll() * m_ailDeflection) + m_input.getTrimmAilR() - m_input.getTrimmAilL()) * m_ailDamage) + (m_lWingDamageCD + m_rWingDamageCD) + (0.55 * Cldr(m_state.m_mach)) * (m_input.getYaw() * m_rudDeflection))
		+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b * (2.0 * Clp(m_state.m_mach) * m_state.m_omega.x + (1.5 * -Clr(m_state.m_mach)) * m_state.m_omega.y);
}

void FlightModel::M_stab()
{
	//set pitch moment-- 
	//"-" vor Cmde eingefügt, da positiver Wert erwartet--//---Cmde von 0.9 auf 0.8 und Cmalpha von 1.15 auf 1.25 auf 1.35 
	//m_pitchup ist Pitch-Up-Moment durch AoA > 15°; m_airframe.autoPilotAltH() ist Pitch due to auto-Pilot Altitude-hold; m_hStabDamage ist horizontal stabilizer integrity in %  
	
	//---------------Alte Version OHNE Beschränkung des Pitch-Inputs auf maximalen Hoehenruder-ausschlag----------------------------------------------------------------------------------
	/*m_moment.z += m_k * CON_mac * (1.35 * (CmalphaNEW(m_state.m_mach) * m_state.m_aoa) + (0.80 * -CmdeNEW(m_state.m_mach)) * (((m_input.getPitch() + m_pitchup) + m_input.getTrimmUp() - m_input.getTrimmDown() + m_airframe.getAutoPilotAltH()) * m_hStabDamage )) 
			+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_mac * CON_mac * ((1.75 * Cmq(m_state.m_mach))*m_state.m_omega.z + (1.45 * Cmadot(m_state.m_mach)) * m_aoaDot);
	*/
	//1.35 * Cmalpha; 1.75 * cmq; 1.45 * cmadot //Werte werden verändert für geringere Resistenz gegenüber Pitch
	// "+ m_CmqStAG" eingefügt

	//----------------NEUE Version mit Ausschlagsbeschränkung auf max Ausschlag Backstick-------------------------------------------------------------------------------------------------
	m_moment.z += m_k * CON_mac * (1.35 * (CmalphaNEW(m_state.m_mach) * m_state.m_aoa) + (-CmdeNEW(m_state.m_mach)) * ((((m_input.getPitch() * CON_hstdUP) + m_pitchup) + m_input.getTrimmUp() - m_input.getTrimmDown() + m_airframe.getAutoPilotAltH()) * m_hStabDamage))
		+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_mac * CON_mac * ((1.75 * Cmq(m_state.m_mach) + m_CmqStAg) * m_state.m_omega.z + ((1.95 * CmadotNEW(m_state.m_mach)) + m_CmaDOTStAg ) * m_aoaDot );

}

void FlightModel::N_stab()
{
	//set yaw moment-- 
	//"Cnda * da" ausgelassen, da wegen gegenläufiger ailerons geringfügig (Buch Seite 114) "Cnp * Pstab" ausgelassen, da ggf. unnötig 
	// "-" vor Cndr eingefügt, da "Rudereffektivität" positiv sein müsste-- "-" vor Cnb eingefügt, da Dämpfung-- "2.5 *" vor Cnr eingefügt für mehr Dämpfung
	// "- (0.75 * m_stallMult)" und "- m_stallMult" eingefügt wegen Stall-Verhalten// statt 0.75 0.95 als m_stallMult eingefügt 
	//moment.y
	
	//----------------------------ALTE Version ohne Beschränkung des Ruder-Ausschlags------------------------------------------------------
	/*m_moment.y += m_q * ((1.5 - ( 0.95 * m_stallMult)) * -Cnb(m_state.m_mach) * m_state.m_beta + -Cndr(m_state.m_mach) * -m_input.getYaw())
		+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b * ((2.5 - m_stallMult) * Cnr(m_state.m_mach) * m_state.m_omega.y);
	*/


	//----------------------------NEUE Version mit Beschränkung des max-Ruderausschlags-----------------------------------------------------
	// ((1.5 - (0.95 * m_stallMult)) // ((2.5 - m_stallMult) * Cnr(m_state.m_mach) * m_state.m_omega.y) ALT
	
	m_moment.y += m_q * ((1.2 - (0.95 * m_stallMult)) * -CnbNEW(m_state.m_mach) * m_state.m_beta + -Cndr(m_state.m_mach) * (-m_input.getYaw() * m_rudDeflection))
		+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b * ((2.1 - m_stallMult) * Cnr(m_state.m_mach) * m_state.m_omega.y);
}


void FlightModel::lift()
{
	//set lift 
	//-- eingefügt am 16.02. als "Lift = 0.5 * p * V² * s * CL
	//approx m_force.y
	// erster Versuch : m_force.y = m_k * (CLmach(m_state.m_mach) + CLa(m_state.m_aoa)); //Lift ist so schon gut ;-)
	//-------------------ursprüngliche Lift-Formel mit statischem Flap-Lift------------------------------------------
	//m_force.y += m_k * (((CLa(m_state.m_mach) * m_state.m_aoa) + ((CLFlaps + CLblc) * m_flapDamage)) * ((m_lWingDamageCL + m_rWingDamageCL) / 2.0 ) ); //+ CLds(m_state.m_mach)); //aktuell nur Lift due to AoA ohne Stab-Lift 
	
	//------------------neue Lift-Formel mit dynamischem Flap-Lift---------------------------------------------------
	m_force.y += m_k * (((CLa(m_state.m_mach) * m_state.m_aoa) + ((CLFlaps + CLblc) * m_flapDamage)) * ((m_lWingDamageCL + m_rWingDamageCL) / 2.0));
}

void FlightModel::drag()
{
	//set drag
	//--eingefügt 16.02. es fehlt noch supersonic drag, gear-drag, flap-drag, brake-drag
	//approx m_force.x negative
	//erster Versuch: m_force.x = -(m_k * (CDmach(m_state.m_mach) + CDa(m_state.m_aoa)
		//+ ((CLmach(m_state.m_mach) + CLa(m_state.m_mach)) * (CLmach(m_state.m_mach) + CLa(m_state.m_mach))) / CON_pi * CON_AR * CON_e));
	// statt 0.85 jetzt 0.80 * CDa(etc) um Alpha-Drag anzupassen.
	m_force.x += -m_k * ((CDmin(m_state.m_mach)) + (0.80 * (CDa(m_state.m_mach) * m_state.m_aoa)) + (CDeng(m_state.m_mach)) + CDGear + CDFlaps + CDBrk + CDBrkCht + CD_OverMach + m_cdminADD + CD_brFlap); // +CDwave + CDi); CDwave und CDi wieder dazu, wenn DRAG geklärt.
}

void FlightModel::sideForce()
{	
	//set side force
	//m_force.z
	//vor m_input.getYaw() ein "-" eingefügt, da eigentlich "-"Yaw richtig-herum ist.
	m_force.z += m_k * ((Cydr(m_state.m_mach) * m_input.getYaw()) + (Cyb(m_state.m_mach) * m_state.m_beta)); //neu eingefügt 28Mar21
}

void FlightModel::thrustForce()
{
	//set thrust force
	//m_force.x positive
	//m_force = Vec3();
	//m_engine.update(123); //neu eingefügt// und wieder zum testen auskommentiert 
	m_force.x += m_engine.updateThrust() * m_airframe.getEngineDamageMult() * m_thinAirMulti; //NEU m_thinAirMulti angefügt für Thrustreduktion ab 94.000 ft.
	//printf("vector %f \n", m_engine.updateThrust()); //neu eingebaut für Ausgabe
}

void FlightModel::calcAeroDeflection()
{
	if (m_airframe.getGearNPosition() > 0.1)
	{
		m_ailDeflection = 2 * CON_aitgu;
	}
	if (m_airframe.getGearNPosition() == 0.0)
	{
		m_ailDeflection = 2 * CON_aitnu;
	}

	if (m_airframe.getGearNPosition() > 0.1)
	{
		m_rudDeflection = CON_RdDefGDR;
	}
	if (m_airframe.getGearNPosition() == 0.0)
	{
		m_rudDeflection = CON_RdDefGUR;
	}

}

void FlightModel::calcLiftFlaps()
{
	CLFlaps = ((CLa_FL2(m_state.m_mach) * m_state.m_aoa) + CL_FlStat(m_state.m_mach)) * m_airframe.getFlapsPosition();

	CLblc = (m_airframe.BLCsystem() * CLFlaps) * 0.8;//NEU * 0.8 eingefügt
}

void FlightModel::pitchStabAugSystem()
{
	if ((m_airframe.getCompressorDamage() >= 0.35) && (m_input.getElectricSystem() == 1.0))
	{
		m_CmqStAg = CmqStAg(m_state.m_mach);
		m_CmaDOTStAg = CmaDOTStAg(m_state.m_mach);
		m_StabAugSys = 0.5;
	}
	else if ((m_airframe.getCompressorDamage() < 0.35) && (m_input.getElectricSystem() == 1.0))
	{
		m_CmqStAg = 0.0;
		m_CmaDOTStAg = 0.0;
		m_StabAugSys = 1.0;
	}
	else if (m_input.getElectricSystem() == 0.0)
	{
		m_CmqStAg = 0.0;
		m_CmaDOTStAg = 0.0;
		m_StabAugSys = 0.0;
	}
}

void FlightModel::addedDrag()
{
	if ((m_state.m_mach >= 0.0664) && (m_state.m_mach <= 1.4680))
	{
		m_cdminADD = CDminAD(m_state.m_mach);
	}
	else
	{
		m_cdminADD = 0.0;
	}
}

void FlightModel::brokenFlapDrag()
{
	if ((m_airframe.osFlapDamage() > 0.0) && (m_airframe.getFlapsPosition() > 0.5) && ((m_state.m_mach > 0.85) || (m_flapBroken == true)))
	{
		CD_brFlap = 0.15;
		m_flapBroken = true;
	}
	else if ((m_airframe.osFlapDamage() > 0.0) && ((m_airframe.getFlapsPosition() >= 0.15) && (m_airframe.getFlapsPosition() <= 0.5)) && ((m_state.m_mach > 1.05) || (m_flapBroken == true)))
	{
		CD_brFlap = 0.15;
		m_flapBroken = true;
	}
	else if (m_airframe.osFlapDamage() == 0.0)
	{
		CD_brFlap = 0.0;
		m_flapBroken = false;
	}
}

void FlightModel::update(double dt)
{
	m_moment = Vec3();
	m_force = Vec3();

	m_aoaDot = (m_state.m_aoa - m_aoaPrevious) / dt;
	m_aoaPrevious = m_state.m_aoa;

	m_scalarVelocity = magnitude(m_state.m_localAirspeed);
	m_scalarVelocitySquared = m_scalarVelocity * m_scalarVelocity;


	m_k = 0.5 * m_state.m_airDensity * m_scalarVelocitySquared * CON_A;
	m_p = 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b;
	m_q = m_k * CON_b;

	m_state.m_mach = m_scalarVelocity / m_state.m_speedOfSound;
	
	M_mcrit = m_state.m_mach / CON_Mcrit; //Mach/Machcrit //neu eingefügt 18.02.2021
	M_mcrit_b = pow(M_mcrit, CON_wdb); //M_mcrit ^ CON_wdb //neu eingefügt 18.02.2021

	CDwave = CON_wda * M_mcrit_b; //neu eingefügt 18.02.2021
	
	CDi = ((CLmach(m_state.m_mach) * CLmach(m_state.m_mach)) / CON_pi * CON_A * CON_e );

	CDGear = CON_GrDD * m_airframe.getGearNPosition();
	CDFlaps = CON_FlpD2 * m_airframe.getFlapsPosition();
	//---OLD CLFlaps---//	CLFlaps = CON_FlpL2 * m_airframe.getFlapsPosition();
	//CLFlaps = (CLa_FL2(m_state.m_mach) * m_state.m_aoa) * m_airframe.getFlapsPosition();
	CDBrk = CON_BrkD * m_airframe.getSpeedBrakePosition();
	CDBrkCht = CON_ChtD * m_airframe.brkChutePosition();
	//CLblc = m_airframe.BLCsystem() * CLFlaps;
	
	L_stab();
	M_stab();
	N_stab();
	lift();
	drag();
	sideForce();
	thrustForce();
	calculateShake(dt);
	calcAeroDeflection();
	calcLiftFlaps();
	pitchStabAugSystem();
	addedDrag();
	brokenFlapDrag();

	//printf("CD broken Flap %f \n", CD_brFlap);
	//printf("CL Total %f \n", m_force.y);
	//----------------function for Pitchup-Factor, Pitchup-force and pitchup-speed--------------------
	if ((m_state.m_aoa >= 0.2617) && (m_airframe.getFlapsPosition() == 0.0) && (m_state.m_mach > 0.26))
	{
		m_pitchup = 1.25 * ((PitAoA(m_state.m_aoa) * PitMult(m_state.m_mach))); //war 0.65 * zu 0.95 zu 1.25 
	}
	else if ((m_state.m_aoa >= 0.2617) && ((m_airframe.getFlapsPosition() > 0.0) || (m_state.m_mach <= 0.26)))
	{
		m_pitchup = 0.95 * ((PitAoA(m_state.m_aoa) * PitMult(m_state.m_mach))); // war 0.45 * zu 0.65 zu 0.95
	}
	else
	{
		m_pitchup = 0.0;
	}
	//printf("m_pitchup %f \n", m_pitchup);
	//printf("AoA %f \n", m_state.m_aoa);
	
	//-------------function for Stall-AoA and Stall-Speed and resulting stall-force----------------------------------------------
	if ((m_state.m_aoa >= 0.2533) && (m_airframe.getFlapsPosition() == 0.0))
	{
		m_stallMult = 2.15 * (StAoA(m_state.m_aoa) * StAoAMulti(m_state.m_mach));//von 1.75 zu 2.25 zu 1.95 zu 1.45 zu 2.01 zu 1.85 zu 2.05 zu 2.15//neuer Multiplikator
	}
	else if ((m_state.m_aoa >= 0.2533) && (m_airframe.getFlapsPosition() > 0.0)) // || (m_state.m_mach <= 0.26)))
	{
		m_stallMult = 1.15 * (StAoA(m_state.m_aoa) * StAoAMulti(m_state.m_mach)); //0.85 zu 1.35 zu 1.15
	}
	else
	{
		m_stallMult = 0.0;
	}
	//printf("m_stallMult %f \n", m_stallMult);

	//------------------ThinAir-Multiplyer-------------------------- // NEU-NEU-NEU
	if (m_airframe.getAltInFeet() < 94000.0)
	{
		m_thinAirMulti = 1.0;
	}
	else
	{
		m_thinAirMulti = ThinAM(m_airframe.getAltInFeet());
	}

	//-------------------OverMach Drag-------------------------------
	if ((m_state.m_mach < 2.22) && (m_airframe.getEASinKnots() < 760.0))
	{
		CD_OverMach = 0.0;
	}
	else
	{
		if ((m_state.m_mach >= 2.22) && (m_airframe.getEASinKnots() < 760.0))
		{
			CD_OverMach = CDOM(m_state.m_mach);
		}
		else if ((m_state.m_mach < 2.22) && (m_airframe.getEASinKnots() >= 760.0))
		{
			CD_OverMach = CDOK(m_airframe.getEASinKnots());
		}
		else if ((m_state.m_mach >= 2.22) && (m_airframe.getEASinKnots() >= 760.0))
		{
			CD_OverMach = CDOK(m_airframe.getEASinKnots());
		}
	}


	//-------------damage-stuff--------------------------------------
	if (m_airframe.getAileronDamage() < 1.0)
	{
		m_ailDamage = m_airframe.getAileronDamage();
	}
	else
	{
		m_ailDamage = 1.0;
	}
	//im Foglenden dann noch Flap-Damage durch Overspeed einbauen!!!
	if (m_airframe.getFlapDamage() < 1.0)
	{
		m_flapDamage = m_airframe.getFlapDamage();
	}
	else
	{
		m_flapDamage = 1.0;
	}

	if (m_airframe.getLWingDamage() < 1.0)
	{
		m_lWingDamageCL = m_airframe.getLWingDamage();
		m_lWingDamageCD = (m_airframe.getLWingDamage()) / 10.0; //Roll-Links ist -, Roll-Rechts ist + 
	}
	else
	{
		m_lWingDamageCL = 1.0;
		m_lWingDamageCD = 0.1;
	}

	if (m_airframe.getRWingDamage() < 1.0)
	{
		m_rWingDamageCL = m_airframe.getRWingDamage();
		m_rWingDamageCD = (-(m_airframe.getRWingDamage())) / 10; //Roll-Links ist +, Roll-Rechts ist - ?? 
	}
	else
	{
		m_rWingDamageCL = 1.0;
		m_rWingDamageCD = -0.1;
	}

	if (m_airframe.getHoriStabDamage() < 1.0)
	{
		m_hStabDamage = m_airframe.getHoriStabDamage();
	}
	else
	{
		m_hStabDamage = 1.0;
	}

	//printf("HStabDamage %f \n", m_hStabDamage);
	/*printf("HStabIntegrity %f \n", m_airframe.getHoriStabDamage());
	printf("WingLIntegrity %f \n", m_airframe.getLWingDamage());
	printf("WingRIntegrity %f \n", m_airframe.getRWingDamage());
	printf("AileronIntegrity %f \n", m_airframe.getAileronDamage());
	printf("TurbineIntegrity %f \n", m_airframe.getTurbineDamage());
	printf("CompressorIntegrity %f \n", m_airframe.getCompressorDamage());*/
	//printf("Roll %f \n", m_input.m_roll);
}

void FlightModel::calculateShake(double& dt)
{


	// cockpit shake calculations
	double shakeAmplitude{ 0.0 };
	double shakeInstGear = 0.0;
	double buffetAmplitude = 0.6;
	double x{ 0.0 };

	// 20 - 28
	double aoa = toDegrees(std::abs(m_state.m_aoa));
	if (m_state.m_mach > 0.15)
		shakeAmplitude = clamp((aoa - 10.0) / 15.0, 0.0, 1.0);
	
	shakeAmplitude *= buffetAmplitude;

	// GEAR CONTRIBUTION

	double gearContinousShake = (m_airframe.getGearLPosition() + m_airframe.getGearRPosition() + m_airframe.getGearNPosition()) / 3.0 * 0.8;


	if (m_airframe.getGearLPosition() > 0.99 || m_airframe.getGearLPosition() < 0.01)
	{
		gearShake = true;
	}
	else if (m_airframe.getGearRPosition() > 0.99 || m_airframe.getGearLPosition() < 0.01)
	{
		gearShake = true;
	}
	else if (m_airframe.getGearNPosition() > 0.99 || m_airframe.getGearLPosition() < 0.01)
	{
		gearShake = true;
	}
	else
	{
		gearShake = false;
	}

	if (gearShake && !prevGearShake)
	{
		m_shakeDuration.startTimer();
		prevGearShake = true;
	}
	else if (!gearShake)
	{
		prevGearShake = false;
	}

	// FLAPS CONTRIBUTION
	double flapsContinousShake = m_airframe.getFlapsPosition() * 0.06;


	// SPEED BRAKES CONTRIBUTION
	double speedBrakesContinousShake = m_airframe.getSpeedBrakePosition() * 0.12;


	m_shakeDuration.updateLoop(dt);

	if (m_shakeDuration.getState())
	{
		shakeInstGear += 0.5;
	}
	//printf(gearShake ? "true" : "false");
	//printf(prevGearShake ? "true" : "false");

	double airspeedScale = clamp((1.0 / 50.0) * (m_scalarVelocity - 50.0), 0.0, 1.0);

	double shakeGroupA = airspeedScale * std::min(flapsContinousShake + gearContinousShake, 0.075);
	double shakeGroupB = airspeedScale * speedBrakesContinousShake;
	double shakeGroupInst = std::min(shakeInstGear, 1.5);

	//printf("shake: %lf\n", shakeAmplitude);

	m_cockpitShake = shakeAmplitude + shakeGroupA + shakeGroupB + shakeGroupInst;
	m_cockpitShake *= 1.3; //CockpitShakeMultiplier von 1.5 auf 1.3

}



