#include <iostream>
#include <cmath>

//#define DEBUG_ON

// Radar gun parameters
const double RADAR_RANGE = 1500.0; // feet
const double BEAM_ANGLE = 15.0; // degrees (assuming 12-18 deg for consumer grade K-band)
// Computation parameters
const int TARGET_VERIFIED_COUNT = 5; // count
const int ITERATION_LIMIT = 500; // count

// Geometry reference
// Right triangle formulas
// https://www.omnicalculator.com/math/right-triangle-side-angle
// Oblique triangle formulas
// https://www.omnicalculator.com/math/law-of-sines
// https://math.libretexts.org/Bookshelves/Algebra/Algebra_and_Trigonometry_1e_(OpenStax)/10%3A_Further_Applications_of_Trigonometry/10.01%3A_Non-right_Triangles_-_Law_of_Sines

using std::cout, std::endl, std::cin, std::round, std::string;
using std::sin, std::cos, std::tan, std::asin, std::acos, std::atan;

// Used in program logic
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double MPH_TO_FPS = 5280/3600;
const double FPS_TO_MPH = 3600/5280;
const double HALF_BEAM_ANGLE_DEG = BEAM_ANGLE/2.0;
const double HALF_BEAM_ANGLE_RAD = DEG_TO_RAD * HALF_BEAM_ANGLE_DEG;

// Min-Max ranges for input validation
const int radarRangeDistMin = 0;
const int radarRangeDistMax = RADAR_RANGE;
const int radarRoadEdgeDistMin = 0;
const int radarRoadEdgeDistMax = (int) radarRangeDistMax * sin(DEG_TO_RAD * 89); // 89 deg because 90 is too high to get any valid speed... formula:  a = c × sin(α)
const int expectedSpeedMin = 10;
const int expectedSpeedMax = 200;
const int sweepsPerCarMin = 1;
const int sweepsPerCarMax = 100;



// Function declarations
bool   checkArgumentFlag(string arg, std::string flag);
bool   getNumericArgument(string arg, double &value, int min = 0, int max = 100);
bool   getNumericArgument(string arg, int &value, int min = 0, int max = 100);
double getNumericInput(string prompt, int min = 0, int max = 100);
double lawOfSinesFindSideC(double sideA, double angleA, double angleC);
void   testRadarConfig(long scan, long off, int verify, long wait);
void   testRadarConfigReset();
bool   testRadarLoop();
int    testRadarDutyCycle();
void   computeTiming(double &secInBeam, int &preferredSweepsPerCar, long &maxLoopTime, long &scanTime, long &offTime, int &verifyCount, long &waitTime);



// So we can be more compatible with Arduino
#define boolean bool
// Headers for functions defined on Arduino
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

long getTargetLoopTime();
long getProjectedLoopTime();
long getScanPlusVerifyTime();
int getProjectedDutyCycle();

// Code from Arduino below
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

// DEFAULT VARIABLE CONFIGURATION OPTIONS
//      RADIATE suggested times with 0 or 1 verify scan - 125/375 (half-second loop) or 250/775 (one-second loop)
//      RADIATE suggested times with 2 or 3 verify scans - 125/875 (one-second loop) or 250/2250 (2.5 second loop)
//                                NOTE minimum time must be greater than (LCD_SCAN_DELAY + LCD_SCAN_TIME)
#define RADIATE_SCAN_TIME 125   // mS duration of radar active scan; (0)=never radiate; (-1)=always on
#define RADIATE_OFF_TIME  875   // mS delay idle between pricessing loop iterations
#define VERIFY_SPEED      2     // >= 1 - Performs "n" additional scans, runs during 'off' time, may affect duty cycle
#define VERIFY_WAIT       125   // mS delay before additional scans.  If VERIFY_SPEED = 0 this has no effect.
#define AUTO_RUN_RADAR    true  // true - start/stop radar automatically; false - control only by serial
#define PRINT_FIRST_SPEED true  // true - print first scan after trigger pulled even if duplicate
#define PRINT_ZERO_SPEED  false // true - print speed values of zero; false - print only values >0

// CONTROL OPTIONS
#define LCD_SCAN_DELAY 10   // mS delay for LCD to stabilize before reading
#define LCD_SCAN_TIME  20   // mS duration to keep scanning LCD for active segments

//Run control options (could be adjusted later, defaults set here)
long radiateOffTime = RADIATE_OFF_TIME;
long radiateScanTime = RADIATE_SCAN_TIME;
boolean autoRunRadar = AUTO_RUN_RADAR;
boolean printZero = PRINT_ZERO_SPEED;
boolean printFirst = PRINT_FIRST_SPEED;
int verifySpeed = VERIFY_SPEED;
long verifyWait = VERIFY_WAIT;

// Returns the ideal loop time (if no over-run)
long getTargetLoopTime()
{
  return radiateScanTime + radiateOffTime;
}

// Returns the projected loop time (with over-run)
long getProjectedLoopTime()
{
  if(getScanPlusVerifyTime() > getTargetLoopTime())
    return getScanPlusVerifyTime();
  else
    return getTargetLoopTime();
}

// Returns the projected scan+verify time
long getScanPlusVerifyTime()
{
  // Some of these values are guesswork for "extra processing" in the code combined with how many times the LCD scan code is called
  return (radiateScanTime + (2*(LCD_SCAN_DELAY + LCD_SCAN_TIME))) + (verifySpeed * (verifyWait + radiateScanTime + (1*(LCD_SCAN_DELAY + LCD_SCAN_TIME))));
}

// Returns the projected worst case duty cycle
int getProjectedDutyCycle()

{
  long totalRadiateTime = radiateScanTime + (verifySpeed * radiateScanTime);

  return (totalRadiateTime*100)/getProjectedLoopTime();
}

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Code from Arduino above



int main(int argc, char* argv[])
{
		// Input data
		double radarRangeDistance = -1; // feet
		double radarRoadEdgeDistance = -1; // feet
		int expectedSpeed = -1; // MPH
		int expectedSpeedFps = -1; // feet per second
		int preferredSweepsPerCar = -1; // count



		// Check if command line parameters included inputs or we should prompt
		for(int x=0; x < argc; x++)
		{
			#ifdef DEBUG_ON
			cout << "CLI: [" << x << "]=" << argv[x] << endl;
			#endif

			if(checkArgumentFlag(argv[x], "-h"))
			{
				cout << endl;
				cout << "Usage: CalculateParameters [options]" << endl;
				cout << endl;
				cout << "  -h    Help/Usage" << endl;
				cout << "  -r    Range distance to target (" << radarRangeDistMin << "-" << radarRangeDistMax << ")" << endl;
				cout << "  -d    Distance from road edge (" << radarRoadEdgeDistMin << "-" << radarRoadEdgeDistMax << ")" << endl;
				cout << "  -s    Speed of traffic (" << expectedSpeedMin << "-" << expectedSpeedMax << ")" << endl;
				cout << "  -n    Number sweeps per car (" << sweepsPerCarMin << "-" << sweepsPerCarMax << ")" << endl;
				cout << endl;

				return EXIT_FAILURE;
			}

			if(checkArgumentFlag(argv[x], "-r"))
			{
				if(radarRangeDistance != -1)
					cout << "WARNING - duplicate argument '-r' specified: " << argv[x] << endl;
				if(!getNumericArgument(argv[x], radarRangeDistance, radarRangeDistMin, radarRangeDistMax))
					cout << "Data validation failed for argument " << argv[x] << "; ignoring." << endl;
			}

			if(checkArgumentFlag(argv[x], "-d"))
			{
				if(radarRoadEdgeDistance != -1)
					cout << "WARNING - duplicate argument '-d' specified: " << argv[x] << endl;
				if(!getNumericArgument(argv[x], radarRoadEdgeDistance, radarRoadEdgeDistMin, radarRoadEdgeDistMax))
					cout << "Data validation failed for argument " << argv[x] << "; ignoring." << endl;
			}

			if(checkArgumentFlag(argv[x], "-s"))
			{
				if(expectedSpeed != -1)
					cout << "WARNING - duplicate argument '-s' specified: " << argv[x] << endl;
				if(!getNumericArgument(argv[x], expectedSpeed, expectedSpeedMin, expectedSpeedMax))
					cout << "Data validation failed for argument " << argv[x] << "; ignoring." << endl;
			}

			if(checkArgumentFlag(argv[x], "-n"))
			{
				if(preferredSweepsPerCar != -1)
					cout << "WARNING - duplicate argument '-n' specified: " << argv[x] << endl;
				if(!getNumericArgument(argv[x], preferredSweepsPerCar, sweepsPerCarMin, sweepsPerCarMax))
					cout << "Data validation failed for argument " << argv[x] << "; ignoring." << endl;
			}
		}



		// Prompt user for data (if neccesary)
		if(radarRangeDistance == -1)
			 radarRangeDistance =    getNumericInput("Enter radar range distance to target (ft)?      ",radarRangeDistMin,radarRangeDistMax);
		else
			 cout << "CLI: radar range distance to target = " << radarRangeDistance << " ft" << endl;

		if(radarRoadEdgeDistance == -1)
			 radarRoadEdgeDistance = getNumericInput("Enter radar distance from edge of road (ft)?    ",radarRoadEdgeDistMin,radarRoadEdgeDistMax);
		else
			 cout << "CLI: radar distance from edge of road = " << radarRoadEdgeDistance << " ft" << endl;

		if(expectedSpeed == -1)
			 expectedSpeed = (int)   getNumericInput("Enter expected maximum speed of traffic (MPH)?  ",expectedSpeedMin,expectedSpeedMax);
		else
			 cout << "CLI: expected maximum speed of traffic = " << expectedSpeed << " MPH" << endl;

		if(preferredSweepsPerCar == -1)
			 preferredSweepsPerCar = (int)   getNumericInput("Enter preferred number of radar sweeps per car? ",sweepsPerCarMin,sweepsPerCarMax);
		else
			 cout << "CLI: preferred number of radar sweeps per car = " << preferredSweepsPerCar << " ft" << endl;



		// Populate converted units
		expectedSpeedFps = round(expectedSpeed * MPH_TO_FPS);
		


		// Warn if inputs are marginal
		if(preferredSweepsPerCar < 2)
		{
			cout << endl;
			cout << "  WARNING - preferred sweeps per car is < 2, may miss vehicles entering mid-sweep!" << endl;
		}

		cout << endl;

		// Calculate road cosine effect angle
		double offsetAngle = 0; // radians
		double offsetAngleDeg = 0; //degrees
		offsetAngle = asin(radarRoadEdgeDistance/radarRangeDistance); // α = arcsin(a / c)
		offsetAngleDeg = RAD_TO_DEG * offsetAngle;
		
		cout << "Radar offset angle will be " << offsetAngleDeg << " deg." << endl;

		if(offsetAngleDeg > 60)
		{
			cout << "  WARNING - cosine effect will be severe at extreme offset angle!" << endl;
		}
		else if(offsetAngleDeg > 40)
		{
			cout << "  NOTE - cosine effect will be high at high offset angle!" << endl;
		}
		else if(offsetAngleDeg < HALF_BEAM_ANGLE_DEG)
		{
			// If the angle of the gun is less-than the half-beam angle, it is functionally always head-on
			offsetAngleDeg = 0; 
			offsetAngle = 0;
		}



		// Calculate beam width parameters
		double halfBeamWidth = 0; // feet
		double beamWidth = 0; // feet
		halfBeamWidth = radarRangeDistance * tan(HALF_BEAM_ANGLE_RAD);// a = b × tan(α)
		beamWidth = 2 * halfBeamWidth;

		cout << "Beam width at range will be " << beamWidth << " ft." << endl;



		// Calculate road beam intersection (distance in range)
		double roadBeamSpan = 0; // feet
		if(offsetAngleDeg < HALF_BEAM_ANGLE_DEG)
		{
			// If the angle of the gun is less-than the half-beam angle, it is functionally always head-on
			roadBeamSpan = RADAR_RANGE; 
		}
		else
		{
			// If the angle of the gun is greater-than the half-beam angle, we need to use triangle trig
			// NOTE - the angle intersecting the road is symmetrical, use same as calculated offsetAngle
			double leftBeamHalfSpan = 0; // feet
			double leftBeamAngleCDeg = 90+HALF_BEAM_ANGLE_DEG;
			double leftBeamAngleBDeg = 90-offsetAngleDeg;
			double leftBeamAngleADeg = 180-leftBeamAngleBDeg-leftBeamAngleCDeg;
			//cout << "Left Beam Triangle: sideA=" << halfBeamWidth << ", angleA=" << leftBeamAngleADeg << ", angleB=" << leftBeamAngleBDeg << ", angleC=" << leftBeamAngleCDeg << endl;
			leftBeamHalfSpan = lawOfSinesFindSideC(halfBeamWidth, // sideA
																						 DEG_TO_RAD*leftBeamAngleADeg, // angleA
																						 DEG_TO_RAD*leftBeamAngleCDeg); // angleC

			double rightBeamHalfSpan = 0; // feet
			double rightBeamAngleCDeg = 90-HALF_BEAM_ANGLE_DEG;
			double rightBeamAngleBDeg = 90-offsetAngleDeg;
			double rightBeamAngleADeg = 180-rightBeamAngleBDeg-rightBeamAngleCDeg;
			//cout << "Right Beam Triangle: sideA=" << halfBeamWidth << ", angleA=" << rightBeamAngleADeg << ", angleB=" << rightBeamAngleBDeg << ", angleC=" << rightBeamAngleCDeg << endl;
			rightBeamHalfSpan = lawOfSinesFindSideC(halfBeamWidth, // sideA
																						 DEG_TO_RAD*rightBeamAngleADeg, // angleA
																						 DEG_TO_RAD*rightBeamAngleCDeg); // angleC

			roadBeamSpan = leftBeamHalfSpan + rightBeamHalfSpan;
		}

		cout << "Beam span along road will be " << roadBeamSpan << " ft." << endl;



		// Calculate time in beam at speed
		double secInBeam = roadBeamSpan / expectedSpeedFps;
		cout << "Expected time in beam at " << expectedSpeed << " MPH is " << secInBeam << " seconds." << endl;

		if(secInBeam < 2)
		{
			cout << "  WARNING - expected time in radar beam is very low, likely to miss samples!" << endl;
		}

		cout << endl;



		// Recommend radar timing
		long maxLoopTime = 0; // mS
		long scanTime = 0; // mS
		long offTime = 0; // mS
		int verifyCount = 0; // count
		long waitTime = 0; // mS

		computeTiming(secInBeam, preferredSweepsPerCar, maxLoopTime, scanTime, offTime, verifyCount, waitTime);

		//Test and tweak
		int iterations = 0;
		int changes = 0;
		testRadarConfig(scanTime, offTime, verifyCount, waitTime);
		while((!testRadarLoop() || testRadarDutyCycle() > 50) && iterations < ITERATION_LIMIT)
		{
			iterations++;

			testRadarConfig(scanTime, offTime, verifyCount, waitTime);

			if(testRadarDutyCycle() > 50 && verifyCount > 0 ||
			   testRadarDutyCycle() > 40 && verifyCount > 2)
			{
				// If the duty cycle is too high
				if(scanTime > 2000)
				{
					#ifdef DEBUG_ON
					cout << "---> Shifting 10mS scanTime to offTime" << endl;
					#endif
					scanTime-=10;
					offTime+=10;
					changes++;
				}
				else
				{
					#ifdef DEBUG_ON
					cout << "----> Reducing verifyCount by 1 (case duty cycle)" << endl;
					#endif
					verifyCount--;
					changes++;
				}
			}
			else if(!testRadarLoop())
			{
				// If the loop ran too long
				if(waitTime >= 500)
				{
					#ifdef DEBUG_ON
					cout << "----> Reducing waitTime by half (case >=500)" << endl;
					#endif
					waitTime = waitTime / 2;
					changes++;
				}
				else if(verifyCount > 2)
				{
					#ifdef DEBUG_ON
					cout << "----> Reducing verifyCount by 1 (case >2 and loop long)" << endl;
					#endif
					verifyCount--;
					changes++;
				}
				else if(waitTime >= 50)
				{
					#ifdef DEBUG_ON
					cout << "----> Reducing waitTime by half (case >=50)" << endl;
					#endif
					waitTime = waitTime / 2;
					changes++;
					if(verifyCount < 3)
					{
						#ifdef DEBUG_ON
						cout << "--------> Increasing verifyCount by 1" << endl;
						#endif
						verifyCount++;
					}
				}
				else if(waitTime < 50 && waitTime > 0)
				{
					#ifdef DEBUG_ON
					cout << "----> Reducing waitTime to zero" << endl;
					#endif
					waitTime = 0;
					changes++;
				}
				else if(preferredSweepsPerCar > 2)
				{
					#ifdef DEBUG_ON
					cout << "----> Reducing preferredSweepsPerCar by 1/4 and recomputing starting point" << endl;
					#endif
					preferredSweepsPerCar = preferredSweepsPerCar * 0.75;
					computeTiming(secInBeam, preferredSweepsPerCar, maxLoopTime, scanTime, offTime, verifyCount, waitTime);
					changes++;
				}
			}
			else
			{
				// If everything seems good
				break;
			}
		}

		//Final test
		testRadarConfig(scanTime, offTime, verifyCount, waitTime);
		if(!testRadarLoop() || testRadarDutyCycle() > 50)
		{
			cout << endl;
			cout << "******************************************" << endl;
			cout << "*                                        *" << endl;
			cout << "*   WARNING - No good solutions found!   *" << endl;
			cout << "*                                        *" << endl;
			cout << "******************************************" << endl;
			cout << endl;
		}

		cout << "Solution computed in " << iterations << " iterations with " << changes << " changes." << endl;
		cout << "Duty cycle: " << testRadarDutyCycle() << "%" << endl;
		cout << "Cycle Time: " << ((scanTime + offTime) / 1000.0) << " seconds" << endl;
		int sweepsPerCar = secInBeam / ((scanTime + offTime) / 1000.0);
		cout << "Sweeps per car: " << sweepsPerCar << endl;

		cout << endl;
		


		// Output config commands for radar
		cout << "Recommended settings for radar program:" << endl;
		cout << "CORR   " << round(offsetAngleDeg) << endl;
		// OFF and SCAN swapped to reduce chances of warnings printing on radar firmware
		cout << "OFF    " << offTime << endl;
		cout << "SCAN   " << scanTime << endl;
		cout << "VERIFY " << verifyCount << endl;
		cout << "WAIT   " << waitTime << endl;

		//Output config commands for radar one-line
		cout << endl;
		cout << "One-Line Solution: ";
		cout << "CORR " << round(offsetAngleDeg) << "\\n";
		// OFF and SCAN swapped to reduce chances of warnings printing on radar firmware
		cout << "OFF " << offTime << "\\n";
		cout << "SCAN " << scanTime << "\\n";
		cout << "VERIFY " << verifyCount << "\\n";
		cout << "WAIT " << waitTime << endl;

		// Done
		return EXIT_SUCCESS;
}

bool checkArgumentFlag(string arg, string flag)
{
	if(arg.length() >= 2)
	{
		return arg.substr(0,2) == flag;
	}
	else
	{
		return false;
	}
}

// Returns true if successful, false if not successful
bool getNumericArgument(string arg, double &value, int min, int max)
{
	string del = "=";
	auto pos = arg.find(del);
	if(pos != string::npos)
	{
		arg.erase(0, pos + del.length());
	}

	double strValue = std::stod(arg);

	if(strValue < min || strValue > max)
	{
		return false;
	}
	else
	{
		value = strValue;
		return true;
	}
}

bool getNumericArgument(string arg, int &value, int min, int max)
{
	double doubleValue = value;
	if(getNumericArgument(arg, doubleValue, min, max))
	{
		value = (int) doubleValue;
		return true;
	}
	else
	{
		return false;
	}
}

double getNumericInput(string prompt, int min, int max)
{
	double value;
	bool isValid = false;

	while(!isValid)
	{
		cout << prompt;
		cin >> value;

		if(cin.fail())
		{
			cout << ">> Invalid input.  Please enter a numberic decimal value." << endl;

			cin.clear(); // reset failbit
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //skip bad input
		}
		else if (value < min || value > max)
		{
			cout << ">> Invalid input.  Value must be between " << min << " and " << max << "." << endl;
		}
		else
		{
			isValid = true;
		}
	}

	return value;
}

double lawOfSinesFindSideC(double sideA, double angleA, double angleC)
{
	return (sideA/sin(angleA))*sin(angleC);
}

void testRadarConfig(long scan, long off, int verify, long wait)
{
	// Set up variables pulled from Arduino code initialization
	radiateOffTime = off;
	radiateScanTime = scan;
	verifySpeed = verify;
	verifyWait = wait;
}

void testRadarConfigReset()
{
	// Pulled from Arduino code initialization
	radiateOffTime = RADIATE_OFF_TIME;
	radiateScanTime = RADIATE_SCAN_TIME;
	verifySpeed = VERIFY_SPEED;
	verifyWait = VERIFY_WAIT;
}

bool testRadarLoop()
{
	// This was copied from Arduino serial input timing warning check
	long remainderTime = getTargetLoopTime() - getProjectedLoopTime();

	// This compares projected times and duty cycle to desired values
	return remainderTime >= 0;
}

int testRadarDutyCycle()
{
	return getProjectedDutyCycle();
}

void computeTiming(double &secInBeam, int &preferredSweepsPerCar, long &maxLoopTime, long &scanTime, long &offTime, int &verifyCount, long &waitTime)
{
	maxLoopTime = (secInBeam * 1000) / preferredSweepsPerCar; //mS
	// Make max time nicer by truncating to nearest sec
	if(maxLoopTime > 1000)
	{
		maxLoopTime = (maxLoopTime / 1000) * 1000;
	}

	// Pick a starting scan time
	scanTime = maxLoopTime / 8;
	// Make scan time more sensible
	if(scanTime < 100)
	{
		scanTime = 100;
	}
	else if (scanTime > 5000)
	{
		scanTime = 5000;
	}

	// Pick the "off" time based on scan time
	offTime = maxLoopTime - scanTime;
	if(offTime < 0)
	{
		offTime = 0;
	}

	// Pick preferred verify count
	verifyCount = TARGET_VERIFIED_COUNT;

	// Pick the "wait" time based on scan time
	waitTime = scanTime;
}
