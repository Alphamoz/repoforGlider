#include <moduleControl/PID.hpp>
#include <DebugTools.hpp>
#define DEBUGTOOLS true
// reset all value (set to surface value)
void PID::Reset()
{
	error = 0;
	lasterror = 0;
	sumerror = 0;
	refference = 0;
	current = 0;
	AW = 0;
	output = 0;
}

void PID::Print()
{
}

/*
param 1 : struct PID that contains current and last values
param 2 : struct PIDconst that contain constants for PID calculation
param 3 : setpoint for PID calculation
param 4 : input for setpoint calculation
param 5 : true : use antiwindup; false : no antiwindup. default false;
output : struct PID with new values (updated after calculation)
*/

PID CalculatePID(PID _temp, PIDconst constants, float setPoint, float input, bool AW = false)
{
	// temporary struct to save updated values;
	PID temp = _temp;
	// set all temp values to 0
	// temp.Reset();

	temp.refference = setPoint;
	temp.current = input;

	// use last error value from parameter
	temp.lasterror = temp.error;

	// special case if input is heading
	if (temp.name == "heading")
	{
		dcout("Heading now is: " << input << "\n");
		// get smallest angle deviation
		// default direction is positive clockwise (cw)
		temp.error = temp.refference - temp.current;

		if (temp.error > 180)
		{
			temp.error -= 360;
		}
		else if (temp.error < -180)
		{
			temp.error += 360;
		}
	}
	else
	{
		temp.error = temp.refference - temp.current;
	}
	// use sum error value from parameter
	temp.sumerror = temp.sumerror + temp.error + temp.AW;
	// std::cout << "PIDError" << temp.error;
	float _P = temp.error * constants.KP;
	float _I = temp.sumerror * constants.KI * constants.SamplingTime;
	float _D = ((temp.error - temp.lasterror) * constants.KD) / constants.SamplingTime;
	float calculatedPID = _P + _I + _D;
	dcout("Calculated PID is: " << calculatedPID);

	// bang bang control
	if (temp.name == "heading")
	{
		if (fabs(temp.error) > 2 && fabs(temp.error) < 5)
		{
			calculatedPID = 50 * (calculatedPID / fabs(calculatedPID));
		}
		else if (fabs(temp.error) < 2 && fabs(temp.error) > 0)
		{
			calculatedPID = 0;
		}
	}

	if (temp.name == "depth" || temp.name == "veloDepth")
	{
		// if (fabs(temp.error) < 0.5)
		// {
		// 	calculatedPID = 0;
		// }
		// added 30 Jan
		dcout("Calculating for: " << temp.name  << "\n");
		temp.output = calculatedPID;
		return temp;
	}

	float middleValue = ((constants.saturation_lower + constants.saturation_upper) / 2);
	float _PID = calculatedPID + middleValue;

	if (_PID > constants.saturation_upper)
		temp.output = constants.saturation_upper;
	else if (_PID < constants.saturation_lower)
		temp.output = constants.saturation_lower;
	else
		temp.output = _PID;

	// percentage is based on upper and lower limit

	temp.outputPercentage = ((temp.output - (middleValue)) / (middleValue)) * 100;

	// std::cout << "PID_OUTPUT:" << temp.output << std::endl;
	// std::cout << "SaturationLower:" << constants.saturation_lower << std::endl;
	// std::cout << "PID_OUTPUT_TEMP:" << temp << std::endl;

	// anti windup
	if (AW)
		temp.AW = (temp.output - _PID) * constants.AW;
	else
		temp.AW = 0;

	return temp;
}

void PIDconst::Print()
{
}

// declare constants values
void PIDconst::Setup(float _KP, float _KI, float _KD, float _AW, float _samplingtime, float _satUpper, float _satLower)
{
	KP = _KP;
	KI = _KI;
	KD = _KD;
	AW = _AW;
	SamplingTime = _samplingtime;
	saturation_upper = _satUpper;
	saturation_lower = _satLower;
};

int PIDconst::loadConfig()
{
	std::ifstream file("configurations/" + name + ".cfg");
	std::stringstream buffer;

	if (file)
	{
		// copy to buffer
		buffer << file.rdbuf();
		file.close();
	}
	// file not found
	else
	{
		return 0;
	}

	std::string line;
	while (std::getline(buffer, line))
	{
		std::istringstream is_line(line);
		if (line[0] == '#')
		{
			// comment line
		}
		else
		{
			std::string key;
			if (std::getline(is_line, key, '='))
			{
				std::string value;
				if (std::getline(is_line, value))
				{
					if (key == "KP")
					{
						std::istringstream(value) >> std::fixed >> std::setprecision(6) >> KP;
					}
					else if (key == "KI")
					{
						std::istringstream(value) >> std::fixed >> std::setprecision(6) >> KI;
					}
					else if (key == "KD")
					{
						std::istringstream(value) >> std::fixed >> std::setprecision(6) >> KD;
					}
					else if (key == "AW")
					{
						std::istringstream(value) >> std::fixed >> std::setprecision(6) >> AW;
					}
					else if (key == "SamplingTime")
					{
						std::istringstream(value) >> std::fixed >> std::setprecision(6) >> SamplingTime;
					}
					else if (key == "saturation_upper")
					{
						std::istringstream(value) >> std::fixed >> std::setprecision(6) >> saturation_upper;
					}
					else if (key == "saturation_lower")
					{
						std::istringstream(value) >> std::fixed >> std::setprecision(6) >> saturation_lower;
					}
				}
			}
		}
	}
	return 1;
};

/*------------------------------------------------------------------------------LEGACY NEW
float PID_heading_rudder(float _yawRefference, float _globalHeading) {
	heading.refference = (_yawRefference*3.1416)/180.0;
	heading.current = (_globalHeading*3.1416)/180.0;

	heading.lasterror = heading.error;
	//need further information
	heading.error = heading.error - heading.current;
	heading.sumerror = heading.sumerror + heading.error + heading.AW;

	float PID_rd_pro = heading.error * rudder.KP;
	float PID_rd_int = heading.sumerror * rudder.KI * rudder.SamplingTime;
	float PID_rd_dev = (heading.error - heading.lasterror) * rudder.KD / rudder.SamplingTime;

	float PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;
	float PID_rd_out;

	if(PID_rd_cal > rudder.saturation_upper) {
		PID_rd_out = rudder.saturation_upper;
	}
	else if(PID_rd_cal < rudder.saturation_lower) {
		PID_rd_out = rudder.saturation_lower;
	}
	else {
		PID_rd_out = PID_rd_cal;
	}

	heading.AW = (PID_rd_out - PID_rd_cal) * rudder.AW;

	return PID_rd_out;
}

float PID_pitch_ballast(float _pitchRefference, float _globalPitch) {
	pitch.refference = _pitchRefference;
	pitch.current = _globalPitch;

	pitch.lasterror = pitch.error;
	pitch.error = pitch.refference - pitch.current;
	pitch.sumerror = pitch.sumerror + pitch.error + pitch.AW;

	float PID_blst_pro = pitch.error * ballast.KP;
	float PID_blst_int = pitch.sumerror * ballast.KI * ballast.SamplingTime;
	float PID_blst_dev = (pitch.error - pitch.lasterror) * ballast.KD / ballast.SamplingTime;

	float PID_blst_cal = PID_blst_pro + PID_blst_int + PID_blst_dev;
	float PID_blst_out;

	if(PID_blst_cal > ballast.saturation_upper) {
		PID_blst_out = ballast.saturation_upper;
	}
	else if(PID_blst_cal < ballast.saturation_lower) {
		PID_blst_out = ballast.saturation_lower;
	}
	else {
		PID_blst_out = PID_blst_cal;
	}

	//	pitch.AW = (PID_blst_out - PID_blst_cal) * ballast.AW;

	return PID_blst_out;
}

float PID_buoyancy_bladder(float _depthRefference, float _globalDepth) {
	buoyancy.refference = _depthRefference;
	buoyancy.current = _globalDepth;

	buoyancy.lasterror = buoyancy.error;
	buoyancy.error = buoyancy.refference - buoyancy.current;
	buoyancy.sumerror = buoyancy.sumerror - buoyancy.error + buoyancy.AW;

	float PID_bld_pro = buoyancy.error * bladder.KP;
	float PID_bld_int = buoyancy.sumerror * bladder.KI * bladder.SamplingTime;
	float PID_bld_dev = (buoyancy.error - buoyancy.lasterror) * bladder.KD / bladder.SamplingTime;

	float PID_bld_cal = PID_bld_pro + PID_bld_int + PID_bld_dev;
	float PID_bld_out;

	if(PID_bld_cal < bladder.saturation_upper) {
		PID_bld_out = bladder.saturation_upper;
	}
	else if(PID_bld_cal > bladder.saturation_lower) {
		PID_bld_out = bladder.saturation_lower;
	}
	else {
		PID_bld_out = PID_bld_cal;
	}

	//depth.AW = (PID_bld_out - PID_bld_cal) * ballast.AW;

	//need further information

	//if(PID_bld_out != 0) {
	//	cout << "PID bld out thread BB : " << PID_bld_out << endl;
	//	buoyancy.lasterror = buoyancy.error;
	//}
	//else if(PID_bld_out == 0) {
	//	std::cout << "PID bld thread BB : 0" << std::endl;
	//}

	return PID_bld_out;
}
------------------------------------------------------------------------------LEGACY NEW*/

/*------------------------------------------------------------------------------LEGACY OLD
void PID_heading_rudder() {
	cout << "PID rudder running" << endl;
	heading_ref = yawref;
	heading_fb = temp_yaw;
	heading_ref = (heading_ref*3.1416)/180.0;
	heading_fb = (heading_fb*3.1416)/180.0;
	heading_laster = heading_err;
	heading_err = heading_err - heading_fb;
	heading_sumerr = heading_sumerr + heading_err + rd_AW;

	PID_rd_pro = heading_err * PID_rd_KP;
	PID_rd_int = heading_sumerr * PID_rd_KI * sampling_time;
	PID_rd_dev = (heading_err - heading_laster) * PID_rd_KD / sampling_time;
	PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;

	if(PID_rd_cal > sat_rd_u) {
		PID_rd_out = sat_rd_u;
	}
	else if(PID_rd_cal < sat_rd_d) {
		PID_rd_out = sat_rd_d;
	}
	else() {
		PID_rd_out = PID_rd_cal;
	}

	rd_AW = (PID_rd_out - PID_rd_cal) * PID_rd_AW;
	rd_PID_val = PID_rd_out;

	if(PID_rd_out != 0) {
		cout << "PID rudder : " << PID_rd_out << endl;
	}
	else if(PID_rd_out == 0) {
		cout << "PID rudder : 0" << endl;
	}
}

void PID_pitch_ballast() {
	cout << "PID ballast running" << endl;
	pitch_ref = ref_pitch;
	printf("pitch_ref : %f\n", pitch_ref);
	pitch_fb = temp_pitch;
	pitch_lasterr = pitch_err;
	pitch_err = pitch_ref - pitch_fb;
	printf("pitch_err : %f\n", pitch_err);
	pitch_sumerr = pitch_sumerr + pitch_err + blst_AW;

	PID_blst_pro = pitch_err * PID_blst_KP;
	PID_blst_int = pitch_sumerr * PID_blst_KI * sampling_time;
	PID_blst_dev = (pitch_err - pitch_lasterr) * PID_blst_KD / sampling_time;
	PID_blst_cal = PID_blst_pro + PID_blst_int + PID_blst_dev;
	printf("blst_cal : %f\n", PID_blst_cal);

	if(PID_blst_cal > sat_blst_u) {
		PID_blst_out = sat_blst_u;
	}
	else if(PID_blst_cal < sat_blst_d) {
		PID_blst_out = sat_blst_d;
	}
	else {
		PID_blst_out = PID_blst_cal;
	}
	// blst_AW = (PID_blst_out - PID_blst_cal) * PID_blst_AW;
	blst_PID_val = PID_blst_out;

	if(PID_blst_out != 0) {
		cout << "PID ballast : " << PID_blst_out << endl;
	}
	else if(PID_blst_out == 0) {
		cout << "PID ballast : 0" << endl;
	}
}

void PID_buoyancy_bladder() {
	cout << "PID bladder running" << endl;
	depth_ref = ref_depth;
	printf("depth_ref %f\n", depth_ref);
	depth_fb = temp_depth;
	depth_lasterr = depth_err;
	depth_err = depth_ref + depth_fb;
	printf("depth_err %f\n", depth_err);
	depth_sumerr = depth_sumerr - depth_err + bld_AW;

	PID_bld_pro = depth_err * PID_bld_KP;
	PID_bld_int = depth_sumerr * PID_bld_KI * sampling_time;
	PID_bld_dev = (depth_err - depth_lasterr) * PID_bld_KD / sampling_time;
	PID_bld_cal = PID_bld_pro + PID_bld_int + PID_bld_dev;
	printf("bld_cal %f\n", PID_bld_cal);

	if(PID_bld_cal < sat_bld_u) {
		PID_bld_out = sat_bld_u;
	}
	else if(PID_bld_cal > sat_bld_d) {
		PID_bld_out = sat_bld_d;
	}
	else {
		PID_bld_out = PID_bld_cal;
	}
	// bld_AW = (PID_bld_out - PID_bld_cal) * PID_bld_AW;
	bld_PID_val = PID_bld_out;

	if(PID_bld_out != 0) {
		cout << "PID bld out thread BB : " << PID_bld_out << endl;
		depth_lasterr = depth_err;
	}
	else if(PID_bld_out == 0) {
		cout << "PID bld thread BB : 0" << endl;
	}
}
------------------------------------------------------------------------------LEGACY OLD*/
