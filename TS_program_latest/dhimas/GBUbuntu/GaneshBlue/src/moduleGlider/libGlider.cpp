#include <moduleGlider/libGlider.hpp>

//load constants values
int GliderConstants::loadConfig()
{
    std::ifstream file ("configurations/gliderConstants.cfg");
    std::stringstream buffer;

    if (file)
    {
        //copy to buffer
        buffer << file.rdbuf();
        file.close();
    }
    //file not found
    else
    {
        return 0;
    }
    
    std::string line;
    while( std::getline(buffer, line) )
    {
        std::istringstream is_line(line);
        if (line[0] == '#')
        {
            //comment line
        } 
        else
        {
            std::string key;
            if( std::getline(is_line, key, '=') )
            {
                std::string value;
                if( std::getline(is_line, value) ) 
                {
                    if (key=="propellerLow")
                    {
						std::istringstream (value) >> propeller[0];
                    } else
                    if (key=="propellerHigh")
                    {
						std::istringstream (value) >> propeller[1];
                    } else
                    if (key=="bowLow")
                    {
                        std::istringstream (value) >> bow[0];
                    } else
                    if (key=="bowHigh")
                    {
                        std::istringstream (value) >> bow[1];
                    } else
                    if (key=="strobeOff")
                    {
                        std::istringstream (value) >> strobe[0];
                    } else
                    if (key=="strobeOn")
                    {
                        std::istringstream (value) >> strobe[1];
                    } else
                    if (key=="ascending")
                    {
                        std::istringstream (value) >> ascending;
                    } else
                    if (key=="descending")
                    {
                        std::istringstream (value) >> descending;
                    } else
                    if (key=="staystill")
                    {
                        std::istringstream (value) >> staystill;
                    } else
                    if (key=="surfaceDepth")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> surfaceDepth;
                    } else
                    if (key=="surfaceThreshold")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> surfaceThreshold;
                    } else
                    if (key=="depthThreshold")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> depthThreshold;
                    } else
                    if (key=="surfacePitch")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> surfacePitch;
                    } else
                    if (key=="descendPitch")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> descendPitch;
                    } else
                    if (key=="ascendPitch")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> ascendPitch;
                    }
                    // Modified 18 Jan 2023
                    if (key=="MMvarHigh")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> MMvarHigh;
                    }
                    if (key=="MMvarLow")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> MMvarLow;
                    }
                    if (key=="BEvarMid")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> BEvarMid;
                    }
                    if (key=="offset")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> offset;
                    }
                    if (key=="rudderMid")
                    {
						std::istringstream (value) >> std::fixed >> std::setprecision(6) >> rudderMid;
                    }
                }
            }
        }   
    }
    return 1;	
}

void GliderConstants::Setup(
        int _propellerLow,
        int _propellerHigh,
        int _bowLow,
        int _bowHigh,
        int _strobeOff,
        int _strobeOn,
        int _ascending,
        int _descending,
        int _staystill,
        float _surfaceDepth,
        float _surfaceThreshold,
        float _depthThreshold,
        float _surfacePitch,
        float _descendPitch,
        float _ascendPitch
    )
{
    propeller[0] = _propellerLow;
    propeller[1] = _propellerHigh;
    bow[0] = _bowLow;
    bow[1] = _bowHigh;
    strobe[0] = _strobeOff;
    strobe[1] = _strobeOn;
    surfaceDepth = _surfaceDepth;
    surfaceThreshold = _surfaceThreshold;
    depthThreshold = _depthThreshold;
    surfacePitch = _surfacePitch;
    descendPitch = _descendPitch;
    ascendPitch = _ascendPitch;
    ascending = _ascending;
    descending = _descending;
    staystill = _staystill; 
}

void GliderConstants::Print()
{
    
}

void Glider::Reset()
{
    descendFlag = false;
    ascendFlag = false;
	pitchUpFlag = false;
    pitchDownFlag = false;
    sensorIsUpdated = false;

    pose = 0;
	guidanceIsFinished = 0;
	strobe = 0;
    datalogCount = 0;
	currentDepth = 0;
	currentPitch = 0;
	depthTarget = 0;
	pos_e = 0;
	pos_n = 0;
	teta_terukur = 0;
	glideAngle = 0;
}

void Glider::Print()
{

}