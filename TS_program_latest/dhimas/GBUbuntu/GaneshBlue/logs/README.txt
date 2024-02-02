//NavigationGuidance
sprintf(logMessageGlide,"%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",	navigation.latitude,navigation.longitude,
                                                                    guidance.distanceFromTarget,guidance.psiRef,
                                                                    guidance.sourceLat,guidance.sourceLon,
                                                                    guidance.targetLat,guidance.targetLon);
Utils::write2LOG("Glide-NavigationGuidance",logMessageGlide,false);

//rudder
sprintf(logMessagePID,"%.6f\t%.6f\t%.6f\t%.6f\r\n",heading.refference,heading.current,heading.error,heading.output);
Utils::write2LOG("PID-heading",logMessagePID,false);

//ballast
sprintf(logMessagePID,"%.6f\t%.6f\t%.6f\t%.6f\r\n",pitch.refference,pitch.current,pitch.error,pitch.output);
Utils::write2LOG("PID-pitch",logMessagePID,false);

//bladder
sprintf(logMessagePID,"%.6f\t%.6f\t%.6f\t%.6f\r\n",buoyancy.refference,buoyancy.current,buoyancy.error,buoyancy.output);
Utils::write2LOG("PID-buoyancy",logMessagePID,false);

//sensor
sprintf(buffer,"%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d %.6f %.6f %.6f %.6f",
    sensor.MINICT.temperature,
    sensor.MINICT.conductivity,
    sensor.ALTI.depth,
    sensor.IMU.latitude,
    sensor.IMU.longitude,
    sensor.IMU.omegaRoll,
    sensor.IMU.omegaPitch,
    sensor.IMU.omegaHeading,
    glider.pose,
    sensor.analog.volt,
    sensor.analog.leak,
    glider.speed,
    glider.boardTemp
);
Utils::write2LOG(&*glider.datalogPath.begin(),buffer,false);