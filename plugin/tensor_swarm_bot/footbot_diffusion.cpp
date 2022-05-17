/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>


/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cGoStraightAngleRange(CRadians(-1.0f),CRadians(1.0f)){}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   try {
   	m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   	m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
    	/* delta: max tolerance for distance */
    	GetNodeAttribute(t_node, "delta", m_cDelta);

    	CDegrees cAngle;
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        m_cSoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        m_cNoTurnAngleThreshold = ToRadians(cAngle);

        m_cGoStraightAngleRange.Set(-m_cNoTurnAngleThreshold, m_cNoTurnAngleThreshold);
        GetNodeAttribute(t_node, "max_speed", m_cMaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot Diffusion controller for robot \"" << GetId() << "\"", ex);
   }
}

/****************************************/
/****************************************/

CVector2 CFootBotDiffusion::DiffusionVector() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_cDelta ) {
      return CVector2::X;
   }
   else {
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}


void CFootBotDiffusion::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_cMaxSpeed);

   /* Wheel speeds */
   Real fSpeed1, fSpeed2;

   if(Abs(cHeadingAngle) > m_cSoftTurnOnAngleThreshold || fHeadingLength < m_cDelta) {
 	/* Hard Turn */
    	fSpeed1 = -m_cMaxSpeed;
        fSpeed2 = m_cMaxSpeed;
   }
   else if(Abs(cHeadingAngle) > m_cNoTurnAngleThreshold) {
       	/* Soft Turn */
        Real fSpeedFactor = (m_cSoftTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_cSoftTurnOnAngleThreshold;
        fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
        fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
   }
   else {
  	/* No Turn: Go Straight */
        fSpeed1 = fBaseAngularWheelSpeed;
        fSpeed2 = fBaseAngularWheelSpeed;
   }

   /* Apply spped for right and left wheels to turn accourding to the state*/
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

void CFootBotDiffusion::ControlStep() {
   /* Get the diffusion vector to perform obstacle avoidance */
   CVector2 cDiffusion = DiffusionVector();

   SetWheelSpeedsFromVector(m_cMaxSpeed * cDiffusion);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
