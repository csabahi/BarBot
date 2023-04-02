/*
University of Waterloo Department of Mechatronics Engineering
EV3 Autonomous Robot Cumulating Project
		The BarBot
		Version 1.0

Camron Sabahi-Pourkashani
Dylan Finlay 
Marlon Poddalgoda
Waren Cao

*/




#include "PC_FileIO.c"

//Global variable initialization

const int DRIVEPOWER = 60;
const int TURNPOWER = 25;
const int RIGHT_ANGLE = 90;
const int TURNAROUND  = 178;
const bool FORCLIENT = true;
const bool FORBAR = false;
const int OBJDETECTLIMIT = 35;

/*unction given by MTE 121 course
that configures the sensors*/
void sensorConfiguration()
{
	SensorType[S1] = sensorEV3_Touch;
	SensorType[S4] = sensorEV3_Ultrasonic;
	SensorType[S2] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S2] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[S2] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
}

/*Trivial function that takes in a distance
in centimeteres and
returns that value in encoder counts*/
float cmToEncoder( float distance )
{
	const float conversion = 180/(PI*1.75);
	return distance * conversion;
}


/*This function accelerates and
decelerates the robot
by passing the motor power,
the robot should accelerate to*/
void accelerate (int motor_power)
{
	if ( motor[motorA] == motor_power )
	{	return; }

	else if(motor_power != 0)
	{
		float i = 8.0;
		while (i >= 1.0)
		{
			motor[motorA] = motor[motorD]
			= (motor_power/i);
			wait1Msec(60);

			if (i>3.0)
			{ i--;}

			else
			{	i -= 0.25;}
		}
	}
	else
	{
		float i = 8.0;
		int current_power = 0;

		current_power = motor[motorA];

		while (i >= 1.0)
		{
			motor[motorA] = motor[motorD]
			= current_power - (current_power/i);
			wait1Msec(80);

			if (i>3.5)
			{ i--; }
			else
			{ i -= 0.25; }
		}
	}
}


/*This function checks for obstacles on
either the left or the right side
depending on the state of the direction boolean*/
bool checkObstacle( bool objectDetected, bool direction)
{
	// true == CCW rotation i.e looks to the left of robot
	// false == CW rotation i.e looks to the right of robot

	int checkDirection = 1;

	const int encoderRotationLimit = 90;

	if (! direction)
	{checkDirection = -1;}
	nMotorEncoder[ motorC ] = 0;
	motor[motorC] = -10 * checkDirection;
	while( abs(nMotorEncoder[motorC]) < encoderRotationLimit)
	{}
	motor[motorC] = 0;

	if (SensorValue[S4] < 30)
	{
		objectDetected = true;
		displayString(5, "OBJECT DETECTED");
	}

	wait1Msec(500);
	motor[motorC] = 10 * checkDirection;
	while ( abs(nMotorEncoder[motorC]) > 0)
	{}

	motor[motorC] = 0;
	return objectDetected;
}

/*This functions runs the routine when the killswitch is pressed
Uses the acceleration function*/
void killSwitchCheck (bool buttonCheck)
{
	// Checks to see if killswitch button has been pressed
	if (getButtonPress(buttonUp))
	{	buttonCheck = true;}

	if (buttonCheck == true)
	{
		/*slows down the robot to a stop and stops the
		rotating ultrasonic and platform motors*/
		setSoundVolume(100);
		playSoundFile("KillSwitchActivated.rsf");
		accelerate(0);

		/* If the scissor lift is raised lower it
		to the original position*/
		if (nMotorEncoder[motorC] != 0)
		{
			motor[motorC] = -10;
			while(nMotorEncoder[motorC] > 0)
			{}
			motor[motorC] = 0;
		}
		motor[motorB]=motor[motorC]=0;

		/*displays to the user that the kill switch is
		pressed then stops the program*/
		displayString(5, "Kill switch pressed");
		displayString(9, "Barbot shutting down");
		setSoundVolume(100);
		playSoundFile("BarbotShuttingDown.rsf");
		wait1Msec(5000);
		stopAllTasks();
	}
}


void adjustBeforeTurn()
{
	if (getGyroDegrees(S2) < 0)
	{
		motor[motorD] = 10;
		motor[motorA] = -10;
		while( getGyroDegrees(S2) < 0)
		{}
		motor[motorA] = motor[motorD] = 0;
	}

	else if (getGyroDegrees(S2) > 0)
	{
		motor[motorA] = 10;
		motor[motorD] = -10;
		while(getGyroDegrees(S2) > 0)
		{}
		motor[motorA] = motor[motorD] = 0;
	}

	wait1Msec(300);

}

/*This function gets the rotation angle, motor power
to turn at and a boolean to determine which direction
the robot should rotate. it also has a killswitch
incorporated in all the different scopes of while
loops and for loops likes in every other function
throughout the project*/
void rotateRobot( int rotationAngle, int turnPower, bool turnCW )
{
	// Passing true for direction will rotate the robot clockwise
	// Passing false for direction will rotate the robot counter clockwise
	int directionFactor = 0;
	bool buttonCheck = false;

	directionFactor = 1;

	if( !turnCW )
	{	directionFactor = -1; }

	adjustBeforeTurn();

	resetGyro(S2);

	motor[ motorA ] = directionFactor * turnPower;
	motor[ motorD ] = directionFactor * turnPower * -1;
	killSwitchCheck(buttonCheck);

	while( (abs( getGyroDegrees( S2 ) ) < rotationAngle)
		&& (!getButtonPress(buttonUp)) )
	{}
	killSwitchCheck(buttonCheck);

	motor[ motorA ] = motor[ motorD ] = 0;

	resetGyro(S2);
}

/*This function checks if the robot overshot its
	marker when it
	drops out of the object avoidance scope/routine*/
void overshoot(int encoder_limit, int current_encoder)
{
	bool turnDirection = true;
	bool buttonCheck = false;
	if (current_encoder > encoder_limit)
	{
		rotateRobot (TURNAROUND, TURNPOWER, turnDirection);

		float dist_drive = current_encoder - encoder_limit;

		nMotorEncoder[motorD] = 0;

		motor[motorA] = motor[motorD] = 50;

		while((nMotorEncoder[motorD] < dist_drive )
			&& (!getButtonPress(buttonUp)))
		{}

		killSwitchCheck(buttonCheck);
		motor[motorA] = motor[motorD] = 0;
		rotateRobot (TURNAROUND, TURNPOWER, turnDirection);
	}
}


/*
This function runs the routine if an object is detected
Uses checkObstacle and rotateRobot function. The function
also return the distance it travelled on the long side of
the recgtangle it drives to adjust the encoderCount
when it breaks out of the function because this function
is used in the driveDist function*/
int isObjectDetect()
{
	const int TURNPOWER = 20;
	const int OBJAVOIDDIST = 45;
	int avoidedObject = 0;
	bool turnCW = true;
	bool objectDetected = false;
	bool buttonCheck = false;

	// reset motor encoder and ensure robot stopped
	nMotorEncoder[motorC] = 0;
	motor[motorA]=motor[motorD] = 0;
	clearTimer(T1);

	playSoundFile("MoveOutOfTheWay.rsf");
	while(time1[T1] < 5000)
	{}

	if (SensorValue[S4] < OBJDETECTLIMIT)
	{
		avoidedObject = 2;
		// check left side
		objectDetected = checkObstacle( objectDetected, true);
		if (objectDetected)
		{turnCW = true;}

		// check right side
		objectDetected = checkObstacle(objectDetected, false);
		if (objectDetected)
		{turnCW = false;}
		setSoundVolume(100);
		playSoundFile("IGoAround.rsf");
		// rotate robot opposite to obstacle detect
		rotateRobot( RIGHT_ANGLE , TURNPOWER, turnCW);

		// drive robot in rectangle around object
		for (int count=0; (count < 2) && (!getButtonPress(buttonUp)); count++)
		{
			nMotorEncoder[motorA] = 0;
			// drive in rectangle
			accelerate(DRIVEPOWER);
			motor[motorA] = motor[motorD] = DRIVEPOWER;
			killSwitchCheck(buttonCheck);

			while( ( nMotorEncoder[motorA] < ( cmToEncoder(OBJAVOIDDIST)*(count+1) ) )
				&& (!getButtonPress(buttonUp)) )
			{}
			killSwitchCheck(buttonCheck);

			motor[motorA] = motor[motorD] = 0;

			// rotate robot
			rotateRobot( RIGHT_ANGLE , TURNPOWER, !turnCW);

			killSwitchCheck(buttonCheck);
		}

		nMotorEncoder[motorA] = 0;
		// drive
		accelerate(DRIVEPOWER);
		motor[motorA] = motor[motorD] = DRIVEPOWER;
		while( (nMotorEncoder[motorA] < cmToEncoder(OBJAVOIDDIST))
			&& (!getButtonPress(buttonUp)) )
		{}
		killSwitchCheck(buttonCheck);
		motor[motorA] = motor[motorD] = 0;

		// turn robot
		rotateRobot( RIGHT_ANGLE , TURNPOWER, turnCW);

		killSwitchCheck(buttonCheck);
	}
	killSwitchCheck(buttonCheck);

	return OBJAVOIDDIST * avoidedObject;
}

void changePlatformH( bool forWho)
{
	const int RAISEPOWER = 10;
	const int ENCODERLIMIT = 894;
	bool buttonCheck = false;

	wait1Msec(1000);
	if (forWho)
	{
		setSoundVolume(100);
		playSoundFile("DrinkThis.rsf");
	}

	nMotorEncoder[ motorB ] = 0;
	motor[motorB] = RAISEPOWER;

	while ((abs(nMotorEncoder[motorB]) < ENCODERLIMIT)
		&& (!getButtonPress(buttonUp)))
	{}
	killSwitchCheck(buttonCheck);
	motor[motorB] = 0;

	// Waits for a tap, this confirms that all the drinks have been removed
	if (forWho)
	{
		setSoundVolume(100);
		playSoundFile("GiveMeMoneyLoud.rsf");
	}

	while (SensorValue[S1] == 0)
	{}
	while(SensorValue[S1] == 1)
	{}
	resetGyro(S2);
	//return motor back to low position
	motor[motorB] = -10;
	while ((abs(nMotorEncoder[motorB]) > 0)
		&& (!getButtonPress(buttonUp)))
	{}
	killSwitchCheck(buttonCheck);
	motor[motorB] = 0;
}


void adjustDrift()
{
	bool buttonCheck = false;

	if (getGyroDegrees(S2) < -2)
	{
		motor[motorD] = DRIVEPOWER + 20;
		while( (getGyroDegrees(S2) < 0)
			&& ( !getButtonPress(buttonUp)))
		{}
		killSwitchCheck(buttonCheck);
		motor[motorD] = DRIVEPOWER;
	}

	if ( getGyroDegrees(S2) > 2 )
	{
		motor[motorD] = DRIVEPOWER - 20;
		while( (getGyroDegrees(S2) > 0)
			&& ( !getButtonPress(buttonUp)))
		{}
		killSwitchCheck(buttonCheck);
		motor[motorD] = DRIVEPOWER;
	}
}


/*This function drives the robot a certain distance
	it uses isOBjectDetect and other functions like acceleration to
	smoothly drive the robot while avoiding objects*/
void driveDist( int dist2Drive, int drivePower )
{
	const int OBJTOL = 50;
	int encoderLimit = 0;
	int encoderCount = 0;
	int prevEncCount = 0;
	bool buttonCheck = false;

	encoderLimit = cmToEncoder(dist2Drive);

	nMotorEncoder[motorA] = 0;

	while( ( encoderCount < encoderLimit )
		&& ( !getButtonPress(buttonUp) ) )
	{
		int avoidedDist = 0;
		accelerate(	drivePower );
		motor[motorA] = motor[motorD] = drivePower;

		if ( (SensorValue[S4] < OBJDETECTLIMIT)
			&& ( !getButtonPress(buttonUp) )
			&& (encoderCount < (encoderLimit - cmToEncoder(OBJTOL) ) ) )
		{
			accelerate(0); // slow down to speed 0 i.e stop
			avoidedDist = isObjectDetect();
			encoderCount += cmToEncoder(avoidedDist);
			prevEncCount = 0;
			nMotorEncoder[motorA] = 0;
			killSwitchCheck(buttonCheck);
			overshoot(encoderLimit, encoderCount);
			nMotorEncoder[motorA] = 0;
		}

		killSwitchCheck(buttonCheck);
		encoderCount += nMotorEncoder[ motorA ] - prevEncCount;
		prevEncCount = nMotorEncoder[ motorA ];
		adjustDrift();
	}

	accelerate(0);
	motor[motorA] = motor[motorD] = 0;
	killSwitchCheck(buttonCheck);
}

/*All the following constants are in centimeters
	This function assumes the distance between tables
	are the same as the table widths and lengths */
void getDistances(int tableNumber, float &xDist, float &yDist)
{
	const int D_BW_TBL = 100;
	const int TABLE_W = 75;
	const int TABLE_L = 75;
	const float HALF = 0.50;
	int row = 0;
	int col = 0;

	row = tableNumber / 10;
	col = tableNumber % 10;
	xDist = row * D_BW_TBL + ( row - 1 ) * TABLE_W - (HALF * D_BW_TBL);
	yDist = col * D_BW_TBL + ( col - 1 ) * TABLE_L + (HALF * D_BW_TBL);
}


void setUpSequence( int housingY )
{
	const int RAISEPOWER = 10;
	const int ENCODERLIMIT = 915;

	wait1Msec(1000);
	nMotorEncoder[ motorB ] = 0;
	motor[motorB] = RAISEPOWER;

	while (abs(nMotorEncoder[motorB]) < ENCODERLIMIT)
	{}
	motor[motorB] = 0;

	wait1Msec(1000);

	motor[motorB] = -10;
	while(abs(nMotorEncoder[motorB]) > 0)
	{}
	motor[motorB] = 0;

	driveDist( housingY, DRIVEPOWER);
	rotateRobot( RIGHT_ANGLE, TURNPOWER, false);
	changePlatformH(FORBAR);
}


task main()
{
	sensorConfiguration();
	resetGyro(S2);

	/*switched all turnCW to the opposite sign ie
	true changed to false and false changed to true
	Constant and variable initialization*/
	const int lengthOfRobot = 40;
	const int HOUSINGY = 20 + (lengthOfRobot/2);
	float xDist = 0;
	float yDist = 0;
	bool turnCW = true;

	//File initialization
	TFileHandle FileIn;
	bool fileOpened = openReadPC(FileIn, "tables_txt.txt");
	if (!fileOpened)
	{
		displayString(5, "File Opening Error");
		wait1Msec(5000);
		stopAllTasks();
	}

	int numOrders = 0;
	readIntPC(FileIn, numOrders);
	displayString(2, "%d tables to deliver to", numOrders);

	setUpSequence( HOUSINGY );

	for( int orderNum = 0; orderNum < numOrders; orderNum++)
	{
		int tableNumber = 0;
		readIntPC( FileIn, tableNumber);
		displayString(7, "Table number %d", tableNumber);
		wait1Msec(500);

		getDistances(tableNumber, xDist, yDist);

		displayString(1, " xDist = %d", xDist);
		displayString(3, "yDist = %d", yDist);

		//These lines drive to the table
		driveDist( xDist , DRIVEPOWER);
		turnCW = true;
		rotateRobot( RIGHT_ANGLE, TURNPOWER, turnCW);
		driveDist( yDist, DRIVEPOWER);

		//Customer Delivery
		changePlatformH(FORCLIENT);

		//Turn around and to return to the bar
		rotateRobot( TURNAROUND, TURNPOWER, turnCW);
		driveDist( yDist, DRIVEPOWER );
		turnCW = false;
		rotateRobot( RIGHT_ANGLE, TURNPOWER, turnCW );
		driveDist( xDist, DRIVEPOWER );

		//Rotate 180 to be facing the tables again
		rotateRobot( TURNAROUND, TURNPOWER, turnCW );
		changePlatformH(FORBAR);
	}

	//Shutdown sequence
	//rotateRobot( TURNAROUND, TURNPOWER, turnCW );
	turnCW = false;
	rotateRobot( RIGHT_ANGLE, TURNPOWER, turnCW);
	driveDist( HOUSINGY, DRIVEPOWER);

	//Turn to be facing forward for next start
	rotateRobot( TURNAROUND, TURNPOWER, turnCW);

	closeFilePC( FileIn );
}
