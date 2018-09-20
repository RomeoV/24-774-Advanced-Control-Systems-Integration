////
//
void doSisoLoopShapingController(float PitchError,float YawError) {
  for (int num_out = 0; num_out < 2; num_out++){
    Vmotors[num_out] = 0.;
    for(int i = 0; i < 5; i++){
      Vmotors[num_out] += C_siso_loop[num_out][i]*internal_state[i];
    }
    Vmotors[num_out] += D_siso_loop[num_out][0] * PitchError + D_siso_loop[num_out][1] * YawError;
  }
  float temp_internal[5] = {0,0,0, 0,0};
  for (int num_states = 0; num_states < 5; num_states++){
    temp_internal[num_states] = 0.;
    for (int i = 0; i < 5; i++){
      temp_internal[num_states] += A_siso_loop[num_states][i]*temp_internal[i];
    }
    temp_internal[num_states] += B_siso_loop[num_states][0] * PitchError + B_siso_loop[num_states][1] * YawError;
  }
  for(int i = 0; i < 5; i++){
    internal_state[i] = temp_internal[i];
  }  
}

void doDidoLoopShapingController(float PitchError,float YawError) {
  for (int num_out = 0; num_out < 2; num_out++){
    Vmotors[num_out] = 0.;
    for(int i = 0; i < 5; i++){
      Vmotors[num_out] += C_dido_loop[num_out][i]*internal_state[i];
    }
    Vmotors[num_out] += D_dido_loop[num_out][0] * PitchError + D_dido_loop[num_out][1] * YawError;
  }
  float temp_internal[5] = {0,0,0, 0,0};
  for (int num_states = 0; num_states < 5; num_states++){
    temp_internal[num_states] = 0.;
    for (int i = 0; i < 5; i++){
      temp_internal[num_states] += A_dido_loop[num_states][i]*temp_internal[i];
    }
    temp_internal[num_states] += B_dido_loop[num_states][0] * PitchError + B_dido_loop[num_states][1] * YawError;
  }
  for(int i = 0; i < 5; i++){
    internal_state[i] = temp_internal[i];
  }  
}

void AeroGains()
{
  //Calculates voltage to be applied to Front and Back Motors K*u
float OutP=0;
float OutY=0;
for (int it=0; it<4;it++){

OutP=OutP+Error[it]*gainP[it];
OutY=OutY+Error[it]*gainY[it];
}
  Vmotors[0]=OutP;
  Vmotors[1]=OutY;
}

//
//
/*
void SetpointGen()
{
  //  Generates setpoints for Pitch and Yaw angles at specific times
  // Note that this is not Real-Time control. The units called seconds should be renamed..
  //
    if (milisecs>=1000){seconds++; milisecs=0;}
     if (seconds >= 4 && seconds <8 ){
      desired[0] = 0.35;
       desired[1] = 0;
      LEDRed = 0;
      LEDGreen = 0;
      LEDBlue = 999;
      return;
      }
      
     if (seconds >= 8 && seconds <12){
       desired[0] = -0.35;
       desired[1] = 0;
      LEDRed = 999;
      LEDGreen = 500;
      LEDBlue = 0;
      return;
      }
 if (seconds >=12 && seconds <16){
      desired[0] = 0;
      desired[0] = 0;
      LEDRed = 0;
      LEDGreen = 0;
      LEDBlue = 999;
      return;
      }

      if (seconds >= 16 && seconds <20 ){
      desired[0] = 0;
       desired[1] = 0.4;
      LEDRed = 999;
      LEDGreen = 500;
      LEDBlue = 0;
      return;
      }
      
     if (seconds >= 20 && seconds <24){
      //seconds=0;
       desired[0] = 0;
       desired[1] = -0.4;
      LEDRed = 0;
      LEDGreen = 0;
      LEDBlue = 999;
      return;
      }
        if (seconds>=24){ 
       seconds=0;
       desired[0] = 0;
       desired[1] = 0;
      LEDRed = 999;
      LEDGreen = 500;
      LEDBlue = 0;
      return;
      }
      
       
      
}
*/
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
