/****************************************************************************************
Course: ROBT1270

Program: SCARA Robot Simulator Intermediate Control

Purpose: To demonstrate intermediate control over the SCARA Robot Simulator by drawing
lines and circles.
Programming methods: formatted I/O, conditional statements, functions, loops,
arrays.

Author: Marcus Kuhn

Declaration: I, Marcus Kuhn,
declare that the following program was written by me.

Date Created: March 28th

****************************************************************************************/
#pragma warning(disable:4996)  // get rid of some microsoft-specific warnings.

//-------------------------- Standard library prototypes --------------------------------
#include <stdio.h>  // formatted i/o
#include <math.h>   // math functions
#include "robot.h"  // robot functions
#include <stdlib.h> // system function
#include <string.h>

CRobot robot;       // the global robot Class.  Can be used everywhere

//---------------------------- Macros ---------------------------------------------------
#define X                     scaraArm.x
#define Y                     scaraArm.y
#define Q1                    scaraArm.theta1
#define Q2                    scaraArm.theta2
#define ARMSOL                scaraArm.armSol
#define PAX                   line.pA.x
#define PAY                   line.pA.y
#define PBX                   line.pB.x
#define PBY                   line.pB.y
#define R                     scaraTool.penColor.r
#define G                     scaraTool.penColor.g
#define B                     scaraTool.penColor.b
#define PENPOS                scaraTool.penPos
#define GREEN                 { 0, 255, 0 }
#define BLACK                 { 0, 0, 0 }
#define BLUE                  { 0, 0, 255 }
#define RED                   { 255, 0, 0 }
#define CMD0                  "scaraMoveJ"
#define CMD0_NARGS            2  // x and y are required
#define SCARAJ_IDX            0
#define CMD1                  "scaraMoveL"
#define CMD1_NARGS            5  // x1, y1, x2, y2 and n are required
#define SCARAL_IDX            1
#define CMD2                  "scaraPenUp"
#define CMD2_NARGS            0  // no argument is required
#define PENUP_IDX             2
#define CMD3                  "scaraPenDown"
#define CMD3_NARGS            0  // no argument is required
#define PENDOWN_IDX           3
#define CMD4                  "scaraMotor"
#define CMD4_NARGS            1  // motorSpeed is required
#define MTRSPEED_IDX          4
#define CMD5                  "scaraPenColor"
#define CMD5_NARGS            3  // r, g and b are required
#define PENCOLOR_IDX          5
#define CMD6                  "quit"
#define CMD6_NARGS            0  // ends the program
#define QUIT_IDX              6
#define ARGS                  scaraCmds[cmdIndex].args
#define DELIM                 " ,\t"

//---------------------------- Program Constants ----------------------------------------
#define PI                    3.14159265358979323846
#define NUM_LINES             8           // number of lines for the main loop
#define MAX_POINTS            50          // maximum number of points in a line
#define MAX_STRING            256         // for the commandString array size
#define LEFT_ARM_SOLUTION     0           // index that can be used to indicate left arm
#define RIGHT_ARM_SOLUTION    1           // index that can be used to indicate right arm
#define L1                    350.0       // inner arm length
#define L2                    250.0       // outer arm length
#define MAX_ABS_THETA1_DEG    150.0       // max angle of inner arm
#define MAX_ABS_THETA2_DEG    170.0       // max angle of outer arm relative to inner arm
#define SLOPE_TOL             (1.0e-5)    // for pen color based on slope
#define MAX_CMDS              7
#define MAX_ARGS              5

//----------------------------- Program Structures Declarations --------------------------------------


typedef struct RGB_COLOR { // for describing pen RGB color
	int r, g, b;   // red, green, blue components (0-255 each)
}RGB_COLOR;

typedef struct POINT_2D {
	double x, y;
}POINT_2D;

typedef struct LINE_DATA {
	POINT_2D pA, pB;  // start point A, end point B
	int numPts; // number of points on line (includes endpoints)
	RGB_COLOR color; // the line color red, green, blue components
}LINE_DATA;

typedef struct SCARA_POS {
	double x, y, theta1, theta2; // TCP coordinate and joint variables
	int armSol;  // right or left arm solution as defined previously in Lab 6
}SCARA_POS;

typedef struct SCARA_TOOL {
	char penPos;    // pen state: up or down
	RGB_COLOR penColor;
}SCARA_TOOL;

typedef struct SCARA_ROBOT {
	SCARA_POS scaraArm{};
	SCARA_TOOL scaraTool{};
	char motorSpeed = 'H'; // H, M or L
}SCARA_ROBOT;

typedef struct CMD {
	const char* name;
	int nArgs; // number of input arguments for a command
	double args[MAX_ARGS]; // arguments
}CMD;

//----------------------------- Local Function Prototypes -------------------------------

double DegToRad(double);  // returns angle in radians from input angle in degrees
double RadToDeg(double);  // returns angle in degrees from input angle in radians
int scaraIK(double, double, double*, double*, int);  // returns if move was valid
SCARA_ROBOT initScaraState(double x, double y, int armSol, SCARA_TOOL penState, char mtrSpeed);  // returns a updated scaraState
int scaraMoveJ(SCARA_ROBOT* scaraState);  // returns if the move was valid or not
void setScaraState(SCARA_ROBOT scaraState);  // scara robot commands function
int pathPlanning(SCARA_ROBOT* allPts, int j);  // chooses either right or left arm solution through pointers, returns if either was available
LINE_DATA initLine(double xA, double yA, double xB, double yB, int numPts);  // returns a updated line data
int scaraMoveL(SCARA_ROBOT* scaraState, LINE_DATA line);  // returns if scaraMoveL was valid
void scaraDisplayState(SCARA_ROBOT scaraState);  // display every variable of the robot
void initScaraCmd(CMD* scaraCmds);
int parseCmd(CMD* scaraCmds, char* cmdLine);
int validateCmd(CMD* scaraCmds, char* cmdName);
int executeCMD(CMD* scaraCmds, int cmdIndex, SCARA_ROBOT* scaraState);
void displayParsing(CMD* scaraCmds, int cmdIndex);
double roundToThousandths(double n);

//---------------------------------------------------------------------------------------

int displayState = 0;  // global variable to be used to optionally display current scaraState...



int main() {
	SCARA_ROBOT scaraState;
	SCARA_TOOL scaraTool;
	LINE_DATA lineData;

	// open connection with robot
	if (!robot.Initialize()) return 0;
	int commandMode = -1;
	puts("Type 1 for set of example lines, type 2 to input a command manually: ");
	scanf_s("%d", &commandMode);
	(void)getchar(); // gets rid of the phantom new line charachter
	switch (commandMode)
	{
	case 1: 	
		// Sets of example lines
		// Initialize a scaraTool
		scaraTool = { 'u', 0, 0, 255 };
		scaraState = initScaraState(300, 300, RIGHT_ARM_SOLUTION, scaraTool, 'H');
		scaraMoveJ(&scaraState);

		// Easy set of lines
		lineData = initLine(300, 0, 300, 300, 10);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(100, 500, 300, 300, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(300, 0, 300, -300, 10);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(100, -500, 300, -300, 20);
		scaraMoveL(&scaraState, lineData);

		// Medium set of lines
		lineData = initLine(0, 600, 600, 0, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(600, 0, 0, -600, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(300, 300, -500, 300, 10);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(-500, 300, -300, 500, 5);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(300, -300, -500, -300, 10);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(-500, -300, -300, -500, 5);
		scaraMoveL(&scaraState, lineData);

		// Hard set of lines
		lineData = initLine(-500, 300, 600, 0, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(600, 0, -300, 500, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(-500, -300, 600, 0, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(600, 0, -300, -500, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(-500, -300, -300, 500, 10);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(-500, 300, -300, -500, 10);
		scaraMoveL(&scaraState, lineData);

		//Challenge set [Bonus]
		lineData = initLine(-500, 300, 0, -600, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(0, -600, -300, 500, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(-500, -300, 0, 600, 20);
		scaraMoveL(&scaraState, lineData);
		lineData = initLine(0, 600, -300, -500, 20);
		scaraMoveL(&scaraState, lineData);

	case 2: 	
		CMD scaraCmds[MAX_CMDS];
		char command[MAX_STRING];
		initScaraCmd(scaraCmds);
		int cmdIndex;
		do {
			printf_s("\nenter a valid scara command or quit> ");
			gets_s(command);
			cmdIndex = parseCmd(scaraCmds, command);
			if (cmdIndex != -1) {
				if (cmdIndex != QUIT_IDX) {
					//displayParsing(scaraCmds, cmdIndex);
					executeCMD(scaraCmds, cmdIndex, &scaraState);
				}
			}
			else
				printf_s("Invalid command.");
		} while (cmdIndex != QUIT_IDX);

	default:
		break;
	}



	//============================= END OF YOUR CODE ==============================

	printf_s("\n\nPress ENTER to end the program...\n");
	//robot.Send("END\n");
	robot.Close(); // close remote connection
	(void)getchar();
	return (0);
}

//-----------------------------Function Definitions ---------------------------------------

/************************************************************************************
* Function: displayParsing
* Purpose: print to the console parsing of command inputted
* arguments:
*   x   -   double
*   y   -   double
*   armSol   -   int
*   penState   -   SCARA_TOOL
*   mtrSpeed   -   char
* return:  updated SCARA_STATE
* Author: Marcus Kuhn
* Date: 20/03/2020
* Modified: 20/03/2020
*************************************************************************************/
void displayParsing(CMD* scaraCmds, int cmdIndex) {
	int i;
	printf_s("\ncommand name> %s", scaraCmds[cmdIndex].name);

	if (cmdIndex == SCARAJ_IDX || cmdIndex == SCARAL_IDX) {      // for scaraMoveJ, nArgs = 2
		for (i = 0; i < scaraCmds[cmdIndex].nArgs; i++)      // loops while token is valid and i is less than the nArgs for the command, same for every following for-loop
			printf_s("\nargument %d> %.2lf", i + 1, ARGS[i]);
	}
	if (cmdIndex == MTRSPEED_IDX) {     // for scaraMotor, nArgs = 1
		for (i = 0; i < scaraCmds[cmdIndex].nArgs; i++)
			printf_s("\nargument %d> %c", i + 1, (char)ARGS[i]);
	}
	if (cmdIndex == PENCOLOR_IDX) {    // for scaraPenColor, nArgs = 3
		for (i = 0; i < scaraCmds[cmdIndex].nArgs; i++)
			printf_s("\nargument %d> %d", i + 1, (int)ARGS[i]);
	}
}

/************************************************************************************
* Function: initScaraCmd
* Purpose: initialize each index of the array with its correspondent string and nArgs
* arguments:
*   scaraCmds   -   CMD
* return:  none
* Author: Marcus Kuhn
* Date: 22/04/2020
* Modified: 22/04/2020
*************************************************************************************/
void initScaraCmd(CMD* scaraCmds) {
	scaraCmds[0].name = CMD0;        // initialize the first command name
	scaraCmds[0].nArgs = CMD0_NARGS; // initialize num of arguments in first command
	scaraCmds[1].name = CMD1;
	scaraCmds[1].nArgs = CMD1_NARGS;
	scaraCmds[2].name = CMD2;
	scaraCmds[2].nArgs = CMD2_NARGS;
	scaraCmds[3].name = CMD3;
	scaraCmds[3].nArgs = CMD3_NARGS;
	scaraCmds[4].name = CMD4;
	scaraCmds[4].nArgs = CMD4_NARGS;
	scaraCmds[5].name = CMD5;
	scaraCmds[5].nArgs = CMD5_NARGS;
	scaraCmds[6].name = CMD6;
	scaraCmds[6].nArgs = CMD6_NARGS;
}

/************************************************************************************
* Function: parseCmd
* Purpose: splits strings into separate tokens for speceific porpuses
* arguments:
*   scaraCmds   -   CMD
*   cmdLine   -   char*
* return:  if command was valid
* Author: Marcus Kuhn
* Date: 22/04/2020
* Modified: 22/04/2020
*************************************************************************************/
int parseCmd(CMD* scaraCmds, char* cmdLine) {
	char* token;  // a pointer variable to a string.
	char* con;
	int i, j = 0;

	// token is assigned the pointer to the token returned by strtok_s
	token = strtok_s(cmdLine, DELIM, &con);

	int cmdIndex = validateCmd(scaraCmds, token);     // finds which index to use for subsequent parsing

	if (cmdIndex != -1 && scaraCmds[cmdIndex].nArgs == 0) {
		token = strtok_s(NULL, DELIM, &con);
		if (token)     // if there is more than one argument, function will also return -1
			cmdIndex = -1;
	}
	if (cmdIndex != -1 && scaraCmds[cmdIndex].nArgs != 0) {
		if (cmdIndex == MTRSPEED_IDX) {     // for scaraMotorSpeed, nArgs = 1
			token = strtok_s(NULL, DELIM, &con);
			// checks if token exists, is of lenght 1 and is alphabetic
			if (token && strlen(token) == 1 && isalpha((int)*token))
				ARGS[0] = (int)*token;            // casts as integer the value (of a char) token is pointing to
			else
				cmdIndex = -1;
			token = strtok_s(NULL, DELIM, &con);
			if (token)     // if there is more than one argument, function will also return -1
				cmdIndex = -1;
		}
		if (cmdIndex == SCARAJ_IDX || cmdIndex == SCARAL_IDX || cmdIndex == PENCOLOR_IDX) {     // for scaraMoveJ, nArgs = 2, for scaraMoveL, nArgs = 5, for scaraPenColor, nArgs = 3
			for (i = 0; token && i < scaraCmds[cmdIndex].nArgs; ++i) {     // loops while token is valid and i is less than the nArgs for the command, same for every following for-loop
				token = strtok_s(NULL, DELIM, &con);
				if (token)                                              // every following token here will be a double
					ARGS[i] = atof(token);            // atof converts to double, then it is stored into the array
				else if (!token)                // token invalid in the range of nArgs
					cmdIndex = -1;
			}
			if (token = strtok_s(NULL, DELIM, &con))     // looks for more than nArgs arguments, which will also make the function return -1
				cmdIndex = -1;
		}
	}
	return cmdIndex;
}

/************************************************************************************
* Function: validadeCmd
* Purpose: uses cmdIndex to execute a scaraCmds[cmdIndex] Scara command.
* It will perform data type casting (double -> int or char) when necessary.
* arguments:
*   scaraCmds   -   CMD*
*   cmdIndex   -   int
*   scaraState   -   SCARA_ROBOT*
* return:  if command was valid
* Author: Marcus Kuhn
* Date: 22/04/2020
* Modified: 22/04/2020
*************************************************************************************/
int validateCmd(CMD* scaraCmds, char* cmdName) {
	int i = 0;
	int idx = -1;
	int invalidCmd = 1;
	while (invalidCmd && i < MAX_CMDS)
		invalidCmd = strcmp(cmdName, scaraCmds[i++].name);
	if (!invalidCmd)
		idx = i - 1;
	return idx;
}

/************************************************************************************
* Function: executeCMD
* Purpose: uses cmdIndex to execute a scaraCmds[cmdIndex] Scara command.
* It will perform data type casting (double -> int or char) when necessary.
* arguments:
*   scaraCmds   -   CMD*
*   cmdIndex   -   int
*   scaraState   -   SCARA_ROBOT*
* return:  if command was valid
* Author: Marcus Kuhn
* Date: 22/04/2020
* Modified: 22/04/2020
*************************************************************************************/
int executeCMD(CMD* scaraCmds, int cmdIndex, SCARA_ROBOT* scaraState) {
	int movePossible = 0;
	switch (cmdIndex) {
	case SCARAJ_IDX:    // scaraMoveJ 
		*scaraState = initScaraState(ARGS[0], ARGS[1], RIGHT_ARM_SOLUTION, scaraState->scaraTool, scaraState->motorSpeed);  // initializes a scaraState with the arguments inputted
		movePossible = scaraMoveJ(scaraState);     // calls function for a joint-interpolated move
		if (movePossible) printf_s("\ninvalid coordinates\n");
		break;
	case SCARAL_IDX:   // scaraMoveL
		LINE_DATA lineData = initLine(ARGS[0], ARGS[1], ARGS[2], ARGS[3], (int)ARGS[4]);      // initialize a lineData for subsequent function call
		movePossible = scaraMoveL(scaraState, lineData);   // calls function for a linear move
		if (movePossible) printf_s("\ninvalid coordinates / number of points\n");
		break;
	case PENUP_IDX:
		scaraState->scaraTool.penPos = 'u';
		setScaraState(*scaraState);
		break;
	case PENDOWN_IDX:
		scaraState->scaraTool.penPos = 'd';
		setScaraState(*scaraState);
		break;
	case MTRSPEED_IDX:
		scaraState->motorSpeed = (char)ARGS[0];     // casting from double to char for motor speed
		if (strpbrk(&scaraState->motorSpeed, "lLmMhH"))     // only calls setScaraState if the motorSpeed command is valid
			setScaraState(*scaraState);
		else
			printf_s("invalid command\n");
		break;
	case PENCOLOR_IDX:
		scaraState->scaraTool.penColor = { (int)ARGS[0], (int)ARGS[1], (int)ARGS[2] };      // casting from double to int for pen colors
		if ((scaraState->R > 0 && scaraState->R < 255) && (scaraState->G > 0 && scaraState->G < 255) && (scaraState->B > 0 && scaraState->B < 255))
			setScaraState(*scaraState);
		else
			printf_s("\ninvalid colors\n");
		break;
	case QUIT_IDX:
		break;
	default:
		break;
	}
	return cmdIndex;
}

/************************************************************************************
* Function: initScaraState
* Purpose: initializes a SCARA_ROBOT variable with x, y, armSol, penState and mtrSpeed
* arguments:
*   x   -   double
*   y   -   double
*   armSol   -   int
*   penState   -   SCARA_TOOL
*   mtrSpeed   -   char
* return:  updated SCARA_STATE
* Author: Marcus Kuhn
* Date: 20/03/2020
* Modified: 20/03/2020
*************************************************************************************/
SCARA_ROBOT initScaraState(double x, double y, int armSol, SCARA_TOOL penState, char mtrSpeed) {
	SCARA_ROBOT scaraStateInit;
	scaraStateInit.X = x;
	scaraStateInit.Y = y;                                                   // initializes every scaraState variable necessary for scaraMoveJ
	scaraStateInit.ARMSOL = armSol;
	scaraStateInit.scaraTool = { penState.penPos, penState.penColor.r, penState.penColor.g, penState.penColor.b };
	scaraStateInit.motorSpeed = mtrSpeed;
	return scaraStateInit;
}

/*---------------------------------------------------------------------------------------
Name: scaraMoveJ
Description: performs joint interpolated move to x,y defined in scaraState.
Calls scaraIK from Lab 6. scaraIK input arguments cannot be changed from Lab 6.
Doe NOT try other armSol if current one doesn't work.
Input Args: pointer to SCARA_ROBOT - state of scaraArm. Assumes x,y, penPos, armSol are defined.
Outputs: scaraState->armPos.theta1, theta2 are updated using scaraState pointer.
Returns: 0 if move was possible with armSol, -1 if it wasn't.
Author: Marcus Kuhn
Date Created: March 2020
---------------------------------------------------------------------------------------*/
int scaraMoveJ(SCARA_ROBOT* scaraState) {
	int validMoveJ = scaraIK(scaraState->X, scaraState->Y, &(scaraState->Q1), &(scaraState->Q2), scaraState->ARMSOL);       // calls scaraIK and stores its return
	if (validMoveJ == 0)
		setScaraState(*scaraState);     // if move was valid, send commands to robot through setScaraState
	return validMoveJ;
}

/************************************************************************************
* Function: scaraIK
* Purpose: performs inverse kinematics, calculating the angles based on desired x and y
* arguments:
*   toolX   -   double
*   toolY   -   double
*   theta1  -  pointer to double
*   theta2  -  pointer to double
*   armSol   -   int
* return:  0 if move was possible, -1 if impossible.
* Author: Marcus Kuhn
* Date: 20/03/2020
* Modified: 20/03/2020
*************************************************************************************/
int scaraIK(double toolX, double toolY, double* theta1, double* theta2, int armSol) {
	double lenght, beta, alpha, angSum, angSumNum, angSumDen;
	int validMove = -1;

	lenght = sqrt(pow(toolX, 2) + pow(toolY, 2));
	lenght = roundToThousandths(lenght);
	if (lenght >= 112.5 && lenght <= 600) {
		beta = atan2(toolY, toolX);															// we need some math
		alpha = acos((pow(L2, 2) - pow(lenght, 2) - pow(L1, 2)) / (-2 * lenght * L1));

		// if armsol == right arm, beta - alpha, else, beta + alpha
		*theta1 = (armSol == RIGHT_ARM_SOLUTION) ? *theta1 = beta - alpha : beta + alpha;

		angSumNum = (toolY - (L1 * sin(*theta1)));		// numerator of angSum (angSum = theta2a)
		angSumDen = (toolX - (L1 * cos(*theta1)));		// denominator
		angSum = atan2(angSumNum, angSumDen);
		*theta2 = angSum - *theta1;

		*theta1 = RadToDeg(*theta1);		// make 'em degrees
		*theta2 = RadToDeg(*theta2);

		if (*theta1 < -210 && *theta1 > -360)
			*theta1 += 360;
		if (*theta1 < 360 && *theta1 > 210)
			*theta1 -= 360;
		//  rotates angles by 360° counter or clock wise if needed
		if (*theta2 < -190 && *theta2 > -360)
			*theta2 += 360;
		if (*theta2 < 360 && *theta2 > 190)
			*theta2 -= 360;

		if (fabs(*theta1) <= MAX_ABS_THETA1_DEG && fabs(*theta2) <= MAX_ABS_THETA2_DEG)
			validMove = 0;
	}
	return validMove;
}

/************************************************************************************
* Function: initLine
* Purpose: initializes a LINE_DATA variable with x1, y1, x2, y2 and numPts
* arguments:
*   xA   -   double
*   yA   -   double
*   xB   -   double
*   yB   -   double
*   numPts   -   int
* return:  updated LINE_DATA.
* Author: Marcus Kuhn
* Date: 20/03/2020
* Modified: 20/03/2020
*************************************************************************************/
LINE_DATA initLine(double xA, double yA, double xB, double yB, int numPts) {
	LINE_DATA line;
	line = { xA, yA, xB, yB, numPts };      // initilizes LINE_DATA and returns a line variable
	return line;
}

/*---------------------------------------------------------------------------------------
Name: scaraMoveL
Description: Move SCARA arm end point with linear motion defined by the line data passed in. The arm is first moved (if needed) to the start point of the line with a call to scaraMoveJ with the pen up.
Input Args: scaraState - state of scaraArm. Assumes x,y, penPos, armSol are defined.
penPos defines if the move draws a line or just performs a linear move. armSol is an initial value and may need to be changed within this function. Path planning occurs within function.
line - the line to move the end of the robot along. penPos in scaraState defines whether a line is drawn with the pen.
Outputs: scaraState->armPos.theta1, theta2, armSol are updated with new scaraState.
Returns: 0 if move was possible, -1 if is impossible.
Author: Marcus Kuhn
---------------------------------------------------------------------------------------*/
int scaraMoveL(SCARA_ROBOT* scaraState, LINE_DATA line) {
	int validMoveL = 0;
	SCARA_TOOL firstPen = { 'u', scaraState->R, scaraState->G, scaraState->B };     // ensures the move to the first point of the line is with the pen up
	SCARA_ROBOT* allPts;
	SCARA_ROBOT armChange;
	allPts = new SCARA_ROBOT[line.numPts];      // creates a dinamically allocated array of size line.numPts of type SCARA_ROBOT.

	if (line.numPts <= MAX_POINTS) {
		for (int i = 0; (i < line.numPts) && (validMoveL == 0); i++) {       // this loop assigns a updated scaraState to every point of the line, as long as all of them are a valid move
			if (i == 0) {    // the first point of the line
				if (atan2(PAY, PAX) > 0.0 && atan2(PAY, PAX) < PI)       // chooses right arm for 1st and 2nd quadrant
					allPts[i].ARMSOL = RIGHT_ARM_SOLUTION;
				else
					allPts[i].ARMSOL = LEFT_ARM_SOLUTION;               // left arm for 3rd and 4th

				if (scaraState->X != PAX || scaraState->Y != PAY) {      // compares current scaraState with desired initial point of the line
					*scaraState = initScaraState(PAX, PAY, allPts[i].ARMSOL, firstPen, scaraState->motorSpeed);
					validMoveL = scaraMoveJ(scaraState);                                                             // moves to first point of the line
				}
				allPts[i].X = PAX;
				allPts[i].Y = PAY;
				allPts[i].Q1 = scaraState->Q1;                      // assigns first point variables to allPts[0] scaraState
				allPts[i].Q2 = scaraState->Q2;
				allPts[i].motorSpeed = scaraState->motorSpeed;
			}
			else {
				double numX = (PBX - PAX);                          //  numerators of distance between first and last point of the line in the X and Y axis equation
				double numY = (PBY - PAY);
				int den = line.numPts - 1;                          //  denomitador of said equations (same for X and Y)
				allPts[i].X = PAX + ((numX / den) * i);
				allPts[i].Y = PAY + ((numY / den) * i);             //  calculates next point of the line and assigns to respective index in the array
				validMoveL = pathPlanning(allPts, i);               //  calls pathPlanning to determine which arm solution to use, validMoveL keeps track if all moves are possible
			}
		allPts[i].scaraTool.penPos = 'd';	// every point of the line is with the pen down, except armChange ones
		}

		for (int k = 1; (k < line.numPts) && (validMoveL == 0); k++) {       //  starting at point one of the line since we already went to point zero in the previous for-loop
			double slope, deltaX, deltaY;
			deltaY = (allPts[k].Y - allPts[k - 1].Y);        // change in Y
			deltaX = (allPts[k].X - allPts[k - 1].X);        // change in X

			slope = (deltaX / deltaY);      // slope of the line (from point k - 1 to k)

			if (deltaX < SLOPE_TOL && deltaX > -SLOPE_TOL)      // if the difference in X approximates 0, set color to black - zero slope
				allPts[k].scaraTool.penColor = BLACK;
			if (deltaY < SLOPE_TOL && deltaY > -SLOPE_TOL)     // if the difference in Y approximates 0, set color to green - infinite slope
				allPts[k].scaraTool.penColor = GREEN;
			else {
				if (slope > SLOPE_TOL)                          //  positive slope gets assigned the blue color
					allPts[k].scaraTool.penColor = BLUE;
				if (slope < -SLOPE_TOL)                         //  negative slope gets assigned the red color
					allPts[k].scaraTool.penColor = RED;
			}
			if (allPts[k].ARMSOL != allPts[k - 1].ARMSOL) {
				armChange = allPts[k - 1];                //  checks if previous arm solution is different than current, if so, lift the pen up as to not draw while changing arm solutions
				armChange.scaraTool.penPos = 'u';
				armChange.ARMSOL = allPts[k].ARMSOL;
				scaraMoveJ(&armChange);
			}
			*scaraState = initScaraState(allPts[k].X, allPts[k].Y, allPts[k].ARMSOL, allPts[k].scaraTool, allPts[k].motorSpeed);
			validMoveL = scaraMoveJ(scaraState);                                                                                    // moves the robot through all the points of the line
		}
	}
	delete[] allPts;       // deallocates the dinamic array after it was used
	return validMoveL;
}

/************************************************************************************
* Function: pathPlanning
* Purpose: Sees if keeping previous arm solution is possible for the next scara robot move, if not tries the other solution.
* Note that if either move is available, it will already be assigned to the desired index in the array.
* arguments:
*   allPts   -   array of SCARA_ROBOTs
*   j   -   integer
* return:  0 if either solution is available, -1 if neither is.
* Author: Marcus Kuhn
* Date: 20/03/2020
* Modified: 20/03/2020
*************************************************************************************/
int pathPlanning(SCARA_ROBOT* allPts, int j) {
	int validPrev;

	allPts[j].ARMSOL = allPts[j - 1].ARMSOL;        //  we always want to keep the arm solution the same as the previous one if possible

	validPrev = scaraIK(allPts[j].X, allPts[j].Y, &allPts[j].Q1, &allPts[j].Q2, allPts[j].ARMSOL);      //  see if keeping the same arm solution is possible

	if (validPrev == -1) {               //  if move is impossible, switch arm solutions and see if the other is possible
		if (allPts[j].ARMSOL == RIGHT_ARM_SOLUTION) {
			allPts[j].ARMSOL = LEFT_ARM_SOLUTION;
			validPrev = scaraIK(allPts[j].X, allPts[j].Y, &allPts[j].Q1, &allPts[j].Q2, allPts[j].ARMSOL);
		}
		else if (allPts[j].ARMSOL == LEFT_ARM_SOLUTION) {
			allPts[j].ARMSOL = RIGHT_ARM_SOLUTION;
			validPrev = scaraIK(allPts[j].X, allPts[j].Y, &allPts[j].Q1, &allPts[j].Q2, allPts[j].ARMSOL);
		}
	}
	return validPrev;   // returns 0 if either solution is available, -1 if neither is. Note that if any is available (with preference to previous one used), it will already be assigned to the desired index in the array.

}

/*---------------------------------------------------------------------------------------
Name: setScaraState
Description: sets motor speed, pen position and calls ROTATE JOINT.
If the global variable called display is 1 then all members of scaraState are displayed
to the console with a call to displayScaraState
Inputs: scaraState - SCARA x,y, theta1,theta2, penPos and motor speed
Outputs: none
Returns: none
Author: Marcus Kuhn
---------------------------------------------------------------------------------------*/
void setScaraState(SCARA_ROBOT scaraState) { // note: not a pointer to scaraState
	char commandString[MAX_STRING];   // string for simulator commands

	if (scaraState.PENPOS == 'u')
		robot.Send("PEN_UP\n");
	else if (scaraState.PENPOS == 'd')
		robot.Send("PEN_DOWN\n");

	if (scaraState.motorSpeed == 'H' || scaraState.motorSpeed == 'h')
		robot.Send("MOTOR_SPEED HIGH\n");
	else if (scaraState.motorSpeed == 'M' || scaraState.motorSpeed == 'm')            // robot simulator commands, sets motor speed, pen position and color, and joint angles defined by the other functions.
		robot.Send("MOTOR_SPEED MEDIUM\n");
	else if (scaraState.motorSpeed == 'L' || scaraState.motorSpeed == 'l')
		robot.Send("MOTOR_SPEED LOW\n");

	sprintf_s(commandString, "PEN_COLOR %d %d %d\n", scaraState.R, scaraState.G, scaraState.B);
	robot.Send(commandString);

	sprintf_s(commandString, "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", scaraState.Q1, scaraState.Q2);
	robot.Send(commandString);

	if (displayState == 1)
		scaraDisplayState(scaraState);
}

//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees
double DegToRad(double angDeg) {
	return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees
double RadToDeg(double angRad) {
	return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------
// Displays every scaraState in the console if displayState is a 1.
void scaraDisplayState(SCARA_ROBOT scaraState) {
	printf_s("\nx> %.2lf\n", scaraState.X);
	printf_s("y> %.2lf\n", scaraState.Y);
	printf_s("theta1> %.2lf\n", scaraState.Q1);
	printf_s("theta2> %.2lf\n", scaraState.Q2);
	printf_s("armSol> %d\n", scaraState.ARMSOL);
	printf_s("penPos> %c\n", scaraState.PENPOS);
	printf_s("R> %d\n", scaraState.R);
	printf_s("G> %d\n", scaraState.G);
	printf_s("B> %d\n", scaraState.B);
	printf_s("motorSpeed> %c\n", scaraState.motorSpeed);
}

/************************************************************************************
 * Function: roundToThousandths
 * - rounds a float to the third decimal place
 *
 * Arguments:
 * n - double
 *
 * return: rounded double
 *
 * Author: Marcus Kuhn
 * Date: 29/04/2020
 * Modified: 29/04/2020
 ***********************************************************************************/
double roundToThousandths(double n) {
	return floor(n * 10000 + 0.5) / 10000;
}