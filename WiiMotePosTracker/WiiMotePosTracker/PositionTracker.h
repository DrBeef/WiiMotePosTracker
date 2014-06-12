#pragma once

#include "wiimote.h"

class PositionTracker
{
public:
	PositionTracker(float remoteSeparationInMM);
	virtual ~PositionTracker(void);

	//Initialises all internal objects - connects to the wiimotes
	bool Initialise();

	//Resets the current position of the sensor bar to be the centre from which the offset is calculated
	bool Reset();

	//Returns the user's head position in millimetres relative to the centre point (which is calculated in Initialise and Reset)
	bool GetPosition(float &x, float &y, float&z);

	void SetRemoteDistance(float remoteDistance);

	//ideally camerass will be horizontal
	void SetCamerasVerticaleAngle(float camerasVerticaleAngle);
	void SetCamerasAngledDownards(bool m_camerasAngledDownards);

	//Provided if a custom IR setup is being used
	void SetIRDotSeparation(float IRDotSeparation);

private:

	wiimote m_remote[2];

	//First calculated X position
	float m_x;
	//First calculated Y position
	float m_y;
	//First calculated Z position
	float m_z;

	//sensor bar width
	float m_IRDotSeparationInMM;

	//dstance between remotes
	float m_remoteSeparationInMM;

	//The angle at which the remote is at (ideally 0)
	float m_camerasVerticaleAngle;

	//Whether the cameras is above natural eye line or below
	bool m_camerasAngledDownards;
};

