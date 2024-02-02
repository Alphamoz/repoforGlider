


struct gpio{
	int Pin;
	FILE* Handler = NULL;                                                             
	char Set[4],String[4],Value[64],Direction[64];                                                                                                                    
}DVL_GPIO,Relay_GPIO;

gpioActivation();

