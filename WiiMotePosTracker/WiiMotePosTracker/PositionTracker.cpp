#include "PositionTracker.h"

#define _USE_MATH_DEFINES
#include <math.h>

struct floatPoint
{
	float x;
	float y;
};

PositionTracker::PositionTracker(float remoteSeparationInMM) :
	//First calculated X position
	m_x(0.0f),
	//First calculated Y position
	m_y(0.0f),
	//First calculated Z position
	m_z(0.0f),
	//sensor bar width mm
	m_IRDotSeparationInMM(8.5f * 25.4f),
	//distance between remotes in mm
	m_remoteSeparationInMM(remoteSeparationInMM),
	//The angle at which the remote is at (ideally 0)
	m_camerasVerticaleAngle(0.0f),
	//Whether the cameras is above natural eye line or below - affects the way the angles are calculated
	m_camerasAngledDownards(false)
{
}
	
PositionTracker::~PositionTracker(void)
{
	//Clean up our act
	if (m_remote[0].IsConnected())
		m_remote[0].Disconnect();
	if (m_remote[1].IsConnected())
		m_remote[1].Disconnect();
}

void PositionTracker::SetRemoteDistance(float remoteSeparationInMM) 
{
	m_remoteSeparationInMM = remoteSeparationInMM;
}

void PositionTracker::SetCamerasVerticaleAngle(float camerasVerticaleAngle)
{
	m_camerasVerticaleAngle = camerasVerticaleAngle;
}

void PositionTracker::SetIRDotSeparation(float IRDotSeparation)
{
	m_IRDotSeparationInMM = IRDotSeparation;
}

void PositionTracker::SetCamerasAngledDownards(bool camerasAngledDownards)
{
	m_camerasAngledDownards = camerasAngledDownards;
}

//Initialises all internal objects - connects to the wiimotes
bool PositionTracker::Initialise()
{
	bool connected = false;
	//Connect to the first remote
	if (m_remote[0].Connect(wiimote::FIRST_AVAILABLE))
	{
		// connected - light first LED
		m_remote[0].SetLEDs(0x01);
		m_remote[0].PlaySquareWave(FREQ_2940HZ);
		Sleep(500);
		m_remote[0].EnableSpeaker (false);

		//Happy with a single connection
		connected = true;
	}

	//Attempt to connect to the second remote
	if (m_remote[1].Connect(wiimote::FIRST_AVAILABLE))
	{
		// connected - light second LED
		m_remote[1].SetLEDs(0x02);
		m_remote[1].PlaySquareWave(FREQ_3130HZ);
		Sleep(500);
		m_remote[1].EnableSpeaker (false);
	}

	//Wait for push of the A button on the first remote
	while(!m_remote[0].Button.A())
	{
		m_remote[0].RefreshState();
		Sleep(100);
	}

	return connected;
}

//Resets the current position of the sensor bar to be the centre from which the offset is calculated
bool PositionTracker::Reset()
{
	//Check we still have a valid connection
	if(m_remote[0].ConnectionLost() ||
		m_remote[1].ConnectionLost())
		return false;

	//Recalculate the absolute location
	m_x=0;
	m_y=0;
	m_z=0;
	return GetPosition(m_x, m_y, m_z);
}


//Returns the user's head position relative to the centre point (which is calculated in Initialise and Reset)
bool PositionTracker::GetPosition(float &x, float &y, float&z)
{
	//Check we still have a valid connection
	if(m_remote[0].ConnectionLost() ||
		m_remote[1].ConnectionLost())
		return false;

	if (m_remote[0].IR.Mode == wiimote_state::ir::OFF ||
		(m_remote[1].IsConnected() && m_remote[1].IR.Mode == wiimote_state::ir::OFF))
		return false;

	m_remote[0].RefreshState();
	if (m_remote[1].IsConnected())
		m_remote[1].RefreshState();

	std::vector<floatPoint> dot[2];

	//Are we able to get a position?
	for (unsigned index=0; index<4; index++)
	{
		if (m_remote[0].IR.Dot[index].bVisible)
		{
			floatPoint p;
			p.x = m_remote[0].IR.Dot[index].X;
			p.y = m_remote[0].IR.Dot[index].Y;
			dot[0].push_back(p);
		}

		if (m_remote[1].IsConnected())
		{
			if (m_remote[1].IR.Dot[index].bVisible)
			{
				floatPoint p;
				p.x = m_remote[1].IR.Dot[index].X;
				p.y = m_remote[1].IR.Dot[index].Y;
				dot[1].push_back(p);
			}
		}
	}

	bool gotPosition = false;
	//if we only have the one remote, then use the Johnny Lee method for distance
	if (!m_remote[1].IsConnected())
	{
		//distance first, requires 2 points
		if (dot[0].size() == 2)
		{
			//If we were warning the user that we had insufficient visible points, then we can stop now
 			if (m_remote[0].IsPlayingAudio())
				m_remote[0].EnableSpeaker(false);

			float dx = dot[0][0].x - dot[0][1].x;
            float dy = dot[0][0].y - dot[0][1].y;
			float pointDist = sqrt(dx * dx + dy * dy);

			float angle = (float)M_PI_4 * pointDist / 2.0f;

            z = ((float)((m_IRDotSeparationInMM / 2) / tan(angle))) - m_z;

            float avgX = (dot[0][0].x + dot[0][1].x) / 2.0f;
            float avgY = (dot[0][0].y + dot[0][1].y) / 2.0f;

            x = (float)(sin((float)M_PI_4 * (avgX - 0.5f)) * z) - m_x;

			float relativeVerticalAngle = ( m_camerasAngledDownards ? -1.0f : 1.0f ) * (avgY - 0.5f) * (float)M_PI_4;
			y = (float)(sin(relativeVerticalAngle + m_camerasVerticaleAngle) * z) - m_y;

			//Hurrah!
			gotPosition = true;
		}
		else
		{
			//Warn user we don't have sufficient (or too many!) IR points with which to do the calculations
			if (!m_remote[0].IsPlayingAudio())
				m_remote[0].PlaySquareWave(FREQ_2940HZ);
		}
	}
	else
	{
		bool dotFail = false;
		if (dot[0].size() == 0)
		{
			if (!m_remote[0].IsPlayingAudio())
				m_remote[0].PlaySquareWave(FREQ_2940HZ);
			dotFail = true;
		}
		else
		{
			if (m_remote[0].IsPlayingAudio())
				m_remote[0].EnableSpeaker(false);
		}

		if (dot[1].size() == 0)
		{
			if (!m_remote[1].IsPlayingAudio())
				m_remote[1].PlaySquareWave(FREQ_3130HZ);
			dotFail = true;
		}
		else
		{
			if (m_remote[1].IsPlayingAudio())
				m_remote[1].EnableSpeaker(false);
		}

		if (dotFail)
		{
			//Nothing else we can do, as we're missing dots from at least one of the remotes
			return false;
		}

		//Now the calcs, we want to get the average point for both remote 1 and 2
		floatPoint remote1Point;
		if (dot[0].size() >= 2)
		{
            remote1Point.x = (dot[0][0].x + dot[0][1].x) / 2.0f;
            remote1Point.y = (dot[0][0].y + dot[0][1].y) / 2.0f;
		}
		else if (dot[0].size() == 1)
		{
            remote1Point.x = dot[0][0].x;
            remote1Point.y = dot[0][0].y;
		}
		else
		{
			//Shouldn't get here
		}
 
		floatPoint remote2Point;
		if (dot[1].size() >= 2)
		{
            remote2Point.x = (dot[1][0].x + dot[1][1].x) / 2.0f;
            remote2Point.y = (dot[1][0].y + dot[1][1].y) / 2.0f;
		}
		else if (dot[1].size() == 1)
		{
            remote2Point.x = dot[1][0].x;
            remote2Point.y = dot[1][0].y;
		}
		else
		{
			//Shouldn't get here
		}

		//First calculate angles for the following scenario ( + is average point, x is dist between remotes)
		//
		//       R1-----x-----R2
		//       /|\         /|\
		//      1   0       1   0
		//
		//
		//
		//
		//
		//           *  +  *

		//Get angles in radians
		float angleR1 = ((3.0f * (float)M_PI) / 8.0f) + (remote1Point.x * (float)M_PI_4);
		float angleR2 = ((3.0f * (float)M_PI) / 8.0f) + ((1.0f - remote2Point.x)  * (float)M_PI_4);

		//Calculate Z
		z = (m_remoteSeparationInMM * sin(angleR1) * sin(angleR2)) / sin((float)M_PI - (angleR1 + angleR2));

		//Use two points to calculate average X
        float r1_x = (float)(sin((float)M_PI_4 * (remote1Point.x - 0.5f)) * z);
        float r2_x = (float)(sin((float)M_PI_4 * (remote2Point.x - 0.5f)) * z);
		x = (r1_x + r2_x) / 2.0f - m_x;

		//Use two points to calculate average Y
		float relativeVerticalAngleR1 = ( m_camerasAngledDownards ? -1.0f : 1.0f ) * (remote1Point.y - 0.5f) * (float)M_PI_4;
		float r1_y = (float)(sin(relativeVerticalAngleR1 + m_camerasVerticaleAngle) * z);

		float relativeVerticalAngleR2 = ( m_camerasAngledDownards ? -1.0f : 1.0f ) * (remote2Point.y - 0.5f) * (float)M_PI_4;
		float r2_y = (float)(sin(relativeVerticalAngleR2 + m_camerasVerticaleAngle) * z);

		y = (r1_y + r2_y) / 2.0f - m_y;

		//Hurrah!
		gotPosition = true;
	}
	
	return gotPosition;
}