////
//
void doDifferentialObserverStep(float PitchRad,float YawRad) {
    for (int el = 0; el < 4; el++) {
         StateX[el] = C_diff_obs[el][0]*ObsInternalState[0] + C_diff_obs[el][1]*ObsInternalState[1] + D_diff_obs[el][0]*PitchRad + D_diff_obs[el][1]*YawRad;
    }
/*
        Serial.println("#################################################################");
        Serial.print("Differential State = ["); 
        Serial.print(StateX[0]); Serial.print(", ");
        Serial.print(StateX[1]); Serial.print(", ");
        Serial.print(StateX[2]); Serial.print(", ");
        Serial.print(StateX[3]); Serial.println("]");
*/
    float tempState0, tempState1;
    tempState0 = A_diff_obs[0][0]*ObsInternalState[0] + A_diff_obs[0][1]*ObsInternalState[1] + B_diff_obs[0][0]*PitchRad + B_diff_obs[0][1]*YawRad;
    tempState1 = A_diff_obs[1][0]*ObsInternalState[0] + A_diff_obs[1][1]*ObsInternalState[1] + B_diff_obs[1][0]*PitchRad + B_diff_obs[1][1]*YawRad;
    ObsInternalState[0] = tempState0;
    ObsInternalState[1] = tempState1;
}

void doLuenbergerObserverStep(float PitchRad,float YawRad, float motor1, float motor2) {
    for (int el = 0; el < 4; el++) StateX[el] = 0.;
    float u[4] = {motor1, motor2, PitchRad, YawRad};

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            StateX[row] += C_luen_obs[row][col]*ObsInternalStateLuen[col] + D_luen_obs[row][col]*u[col];
        }
    }
/*
        Serial.println("#################################################################");
        Serial.print("Luenberger State = ["); 
        Serial.print(StateX[0]); Serial.print(", ");
        Serial.print(StateX[1]); Serial.print(", ");
        Serial.print(StateX[2]); Serial.print(", ");
        Serial.print(StateX[3]); Serial.println("]");
*/

    float tempState[4] = {0,0,0,0};
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            tempState[row] += A_luen_obs[row][col]*ObsInternalStateLuen[col] + B_luen_obs[row][col]*u[col];
        }
    }
    for (int el = 0; el < 4; el++) ObsInternalStateLuen[el] = tempState[el];

}

void AeroGains()
{
    //Calculates voltage to be applied to Front and Back Motors K*u
    float OutP=0;
    float OutY=0;
    for (int el = 0; el < 4; el++) {
        OutP += K[0][el]*Error[el];
        OutY += K[1][el]*Error[el];
    }
    Vmotors[0]=OutP;
    Vmotors[1]=OutY;
}

//
//
void SetpointGen()
{
    //  Generates setpoints for Pitch and Yaw angles at specific times
    // Note that this is not Real-Time control. The units called seconds should be renamed..
    //
/*
    desired[0] = 0.5;
    desired[1] = 0;
    LEDRed = 999;
    LEDGreen = 0;
    LEDBlue = 999;
    return;
*/
    unsigned long current_time_difference_millis = millis() - starting_time_millis;

// PITCH REFERENCE 
    unsigned long T_pitch = 2*M_PI*1000./0.4;
    unsigned long current_phase_pitch_millis = current_time_difference_millis%T_pitch;
    if (current_phase_pitch_millis <= T_pitch/2.) {
        desired[0] = -M_PI/6.0;
    }
    else {
        desired[0] = M_PI/6.0;
    }

// YAW REFERENCE 
    unsigned long T_yaw = 2*M_PI*1000./0.5;
    unsigned long current_phase_yaw_millis = current_time_difference_millis%T_yaw;
    if (current_phase_yaw_millis <= T_yaw/2.) {
        desired[1] = -M_PI/4.0;
    }
    else {
        desired[1] = M_PI/4.0;
    }

    desired[0] = -M_PI/6.0;
    desired[1] = -M_PI/4.0;
    

}
