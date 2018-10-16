  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 7;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (System_inverted_P)
    ;%
      section.nData     = 4;
      section.data(4)  = dumData; %prealloc
      
	  ;% System_inverted_P.gain
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_P.lqr_k
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% System_inverted_P.BandLimitedWhiteNoise1_Cov
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 5;
	
	  ;% System_inverted_P.BandLimitedWhiteNoise1_seed
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% System_inverted_P.HILReadEncoderTimebase_clock
	  section.data(1).logicalSrcIdx = 4;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% System_inverted_P.HILReadEncoderTimebase_channels
	  section.data(1).logicalSrcIdx = 5;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_P.HILWriteAnalog_channels
	  section.data(2).logicalSrcIdx = 6;
	  section.data(2).dtTransOffset = 2;
	
	  ;% System_inverted_P.HILReadEncoderTimebase_samples_
	  section.data(3).logicalSrcIdx = 7;
	  section.data(3).dtTransOffset = 3;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(3) = section;
      clear section
      
      section.nData     = 32;
      section.data(32)  = dumData; %prealloc
      
	  ;% System_inverted_P.HILInitialize_OOTerminate
	  section.data(1).logicalSrcIdx = 8;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_P.HILInitialize_OOExit
	  section.data(2).logicalSrcIdx = 9;
	  section.data(2).dtTransOffset = 1;
	
	  ;% System_inverted_P.HILInitialize_OOStart
	  section.data(3).logicalSrcIdx = 10;
	  section.data(3).dtTransOffset = 2;
	
	  ;% System_inverted_P.HILInitialize_OOEnter
	  section.data(4).logicalSrcIdx = 11;
	  section.data(4).dtTransOffset = 3;
	
	  ;% System_inverted_P.HILInitialize_AOFinal
	  section.data(5).logicalSrcIdx = 12;
	  section.data(5).dtTransOffset = 4;
	
	  ;% System_inverted_P.HILInitialize_OOFinal
	  section.data(6).logicalSrcIdx = 13;
	  section.data(6).dtTransOffset = 5;
	
	  ;% System_inverted_P.HILInitialize_AIHigh
	  section.data(7).logicalSrcIdx = 14;
	  section.data(7).dtTransOffset = 8;
	
	  ;% System_inverted_P.HILInitialize_AILow
	  section.data(8).logicalSrcIdx = 15;
	  section.data(8).dtTransOffset = 9;
	
	  ;% System_inverted_P.HILInitialize_AOHigh
	  section.data(9).logicalSrcIdx = 16;
	  section.data(9).dtTransOffset = 10;
	
	  ;% System_inverted_P.HILInitialize_AOLow
	  section.data(10).logicalSrcIdx = 17;
	  section.data(10).dtTransOffset = 11;
	
	  ;% System_inverted_P.HILInitialize_AOInitial
	  section.data(11).logicalSrcIdx = 18;
	  section.data(11).dtTransOffset = 12;
	
	  ;% System_inverted_P.HILInitialize_AOWatchdog
	  section.data(12).logicalSrcIdx = 19;
	  section.data(12).dtTransOffset = 13;
	
	  ;% System_inverted_P.HILInitialize_OOInitial
	  section.data(13).logicalSrcIdx = 20;
	  section.data(13).dtTransOffset = 14;
	
	  ;% System_inverted_P.HILInitialize_OOWatchdog
	  section.data(14).logicalSrcIdx = 21;
	  section.data(14).dtTransOffset = 17;
	
	  ;% System_inverted_P.Pendulumcountstorad_Gain
	  section.data(15).logicalSrcIdx = 22;
	  section.data(15).dtTransOffset = 20;
	
	  ;% System_inverted_P.Armcountstorad_Gain
	  section.data(16).logicalSrcIdx = 23;
	  section.data(16).dtTransOffset = 21;
	
	  ;% System_inverted_P.Constant_Value
	  section.data(17).logicalSrcIdx = 24;
	  section.data(17).dtTransOffset = 22;
	
	  ;% System_inverted_P.WhiteNoise_Mean
	  section.data(18).logicalSrcIdx = 25;
	  section.data(18).dtTransOffset = 23;
	
	  ;% System_inverted_P.WhiteNoise_StdDev
	  section.data(19).logicalSrcIdx = 26;
	  section.data(19).dtTransOffset = 24;
	
	  ;% System_inverted_P.Internal_A
	  section.data(20).logicalSrcIdx = 27;
	  section.data(20).dtTransOffset = 25;
	
	  ;% System_inverted_P.Internal_B
	  section.data(21).logicalSrcIdx = 28;
	  section.data(21).dtTransOffset = 26;
	
	  ;% System_inverted_P.Internal_C
	  section.data(22).logicalSrcIdx = 29;
	  section.data(22).dtTransOffset = 27;
	
	  ;% System_inverted_P.Internal_D
	  section.data(23).logicalSrcIdx = 30;
	  section.data(23).dtTransOffset = 28;
	
	  ;% System_inverted_P.Internal_InitialCondition
	  section.data(24).logicalSrcIdx = 31;
	  section.data(24).dtTransOffset = 29;
	
	  ;% System_inverted_P.Internal_A_f
	  section.data(25).logicalSrcIdx = 32;
	  section.data(25).dtTransOffset = 30;
	
	  ;% System_inverted_P.Internal_B_o
	  section.data(26).logicalSrcIdx = 33;
	  section.data(26).dtTransOffset = 31;
	
	  ;% System_inverted_P.Internal_C_k
	  section.data(27).logicalSrcIdx = 34;
	  section.data(27).dtTransOffset = 32;
	
	  ;% System_inverted_P.Internal_D_b
	  section.data(28).logicalSrcIdx = 35;
	  section.data(28).dtTransOffset = 33;
	
	  ;% System_inverted_P.Internal_InitialCondition_b
	  section.data(29).logicalSrcIdx = 36;
	  section.data(29).dtTransOffset = 34;
	
	  ;% System_inverted_P.Saturation_UpperSat
	  section.data(30).logicalSrcIdx = 37;
	  section.data(30).dtTransOffset = 35;
	
	  ;% System_inverted_P.Saturation_LowerSat
	  section.data(31).logicalSrcIdx = 38;
	  section.data(31).dtTransOffset = 36;
	
	  ;% System_inverted_P.ForveCCW_Gain
	  section.data(32).logicalSrcIdx = 39;
	  section.data(32).dtTransOffset = 37;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(4) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% System_inverted_P.HILInitialize_CKChannels
	  section.data(1).logicalSrcIdx = 40;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_P.HILInitialize_DOWatchdog
	  section.data(2).logicalSrcIdx = 41;
	  section.data(2).dtTransOffset = 1;
	
	  ;% System_inverted_P.HILInitialize_EIInitial
	  section.data(3).logicalSrcIdx = 42;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(5) = section;
      clear section
      
      section.nData     = 6;
      section.data(6)  = dumData; %prealloc
      
	  ;% System_inverted_P.HILInitialize_AIChannels
	  section.data(1).logicalSrcIdx = 43;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_P.HILInitialize_AOChannels
	  section.data(2).logicalSrcIdx = 44;
	  section.data(2).dtTransOffset = 1;
	
	  ;% System_inverted_P.HILInitialize_DOChannels
	  section.data(3).logicalSrcIdx = 45;
	  section.data(3).dtTransOffset = 2;
	
	  ;% System_inverted_P.HILInitialize_EIChannels
	  section.data(4).logicalSrcIdx = 46;
	  section.data(4).dtTransOffset = 3;
	
	  ;% System_inverted_P.HILInitialize_EIQuadrature
	  section.data(5).logicalSrcIdx = 47;
	  section.data(5).dtTransOffset = 5;
	
	  ;% System_inverted_P.HILInitialize_OOChannels
	  section.data(6).logicalSrcIdx = 48;
	  section.data(6).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(6) = section;
      clear section
      
      section.nData     = 37;
      section.data(37)  = dumData; %prealloc
      
	  ;% System_inverted_P.HILInitialize_Active
	  section.data(1).logicalSrcIdx = 49;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_P.HILInitialize_AOTerminate
	  section.data(2).logicalSrcIdx = 50;
	  section.data(2).dtTransOffset = 1;
	
	  ;% System_inverted_P.HILInitialize_AOExit
	  section.data(3).logicalSrcIdx = 51;
	  section.data(3).dtTransOffset = 2;
	
	  ;% System_inverted_P.HILInitialize_DOTerminate
	  section.data(4).logicalSrcIdx = 52;
	  section.data(4).dtTransOffset = 3;
	
	  ;% System_inverted_P.HILInitialize_DOExit
	  section.data(5).logicalSrcIdx = 53;
	  section.data(5).dtTransOffset = 4;
	
	  ;% System_inverted_P.HILInitialize_POTerminate
	  section.data(6).logicalSrcIdx = 54;
	  section.data(6).dtTransOffset = 5;
	
	  ;% System_inverted_P.HILInitialize_POExit
	  section.data(7).logicalSrcIdx = 55;
	  section.data(7).dtTransOffset = 6;
	
	  ;% System_inverted_P.HILInitialize_CKPStart
	  section.data(8).logicalSrcIdx = 56;
	  section.data(8).dtTransOffset = 7;
	
	  ;% System_inverted_P.HILInitialize_CKPEnter
	  section.data(9).logicalSrcIdx = 57;
	  section.data(9).dtTransOffset = 8;
	
	  ;% System_inverted_P.HILInitialize_CKStart
	  section.data(10).logicalSrcIdx = 58;
	  section.data(10).dtTransOffset = 9;
	
	  ;% System_inverted_P.HILInitialize_CKEnter
	  section.data(11).logicalSrcIdx = 59;
	  section.data(11).dtTransOffset = 10;
	
	  ;% System_inverted_P.HILInitialize_AIPStart
	  section.data(12).logicalSrcIdx = 60;
	  section.data(12).dtTransOffset = 11;
	
	  ;% System_inverted_P.HILInitialize_AIPEnter
	  section.data(13).logicalSrcIdx = 61;
	  section.data(13).dtTransOffset = 12;
	
	  ;% System_inverted_P.HILInitialize_AOPStart
	  section.data(14).logicalSrcIdx = 62;
	  section.data(14).dtTransOffset = 13;
	
	  ;% System_inverted_P.HILInitialize_AOPEnter
	  section.data(15).logicalSrcIdx = 63;
	  section.data(15).dtTransOffset = 14;
	
	  ;% System_inverted_P.HILInitialize_AOStart
	  section.data(16).logicalSrcIdx = 64;
	  section.data(16).dtTransOffset = 15;
	
	  ;% System_inverted_P.HILInitialize_AOEnter
	  section.data(17).logicalSrcIdx = 65;
	  section.data(17).dtTransOffset = 16;
	
	  ;% System_inverted_P.HILInitialize_AOReset
	  section.data(18).logicalSrcIdx = 66;
	  section.data(18).dtTransOffset = 17;
	
	  ;% System_inverted_P.HILInitialize_DOPStart
	  section.data(19).logicalSrcIdx = 67;
	  section.data(19).dtTransOffset = 18;
	
	  ;% System_inverted_P.HILInitialize_DOPEnter
	  section.data(20).logicalSrcIdx = 68;
	  section.data(20).dtTransOffset = 19;
	
	  ;% System_inverted_P.HILInitialize_DOStart
	  section.data(21).logicalSrcIdx = 69;
	  section.data(21).dtTransOffset = 20;
	
	  ;% System_inverted_P.HILInitialize_DOEnter
	  section.data(22).logicalSrcIdx = 70;
	  section.data(22).dtTransOffset = 21;
	
	  ;% System_inverted_P.HILInitialize_DOReset
	  section.data(23).logicalSrcIdx = 71;
	  section.data(23).dtTransOffset = 22;
	
	  ;% System_inverted_P.HILInitialize_EIPStart
	  section.data(24).logicalSrcIdx = 72;
	  section.data(24).dtTransOffset = 23;
	
	  ;% System_inverted_P.HILInitialize_EIPEnter
	  section.data(25).logicalSrcIdx = 73;
	  section.data(25).dtTransOffset = 24;
	
	  ;% System_inverted_P.HILInitialize_EIStart
	  section.data(26).logicalSrcIdx = 74;
	  section.data(26).dtTransOffset = 25;
	
	  ;% System_inverted_P.HILInitialize_EIEnter
	  section.data(27).logicalSrcIdx = 75;
	  section.data(27).dtTransOffset = 26;
	
	  ;% System_inverted_P.HILInitialize_POPStart
	  section.data(28).logicalSrcIdx = 76;
	  section.data(28).dtTransOffset = 27;
	
	  ;% System_inverted_P.HILInitialize_POPEnter
	  section.data(29).logicalSrcIdx = 77;
	  section.data(29).dtTransOffset = 28;
	
	  ;% System_inverted_P.HILInitialize_POStart
	  section.data(30).logicalSrcIdx = 78;
	  section.data(30).dtTransOffset = 29;
	
	  ;% System_inverted_P.HILInitialize_POEnter
	  section.data(31).logicalSrcIdx = 79;
	  section.data(31).dtTransOffset = 30;
	
	  ;% System_inverted_P.HILInitialize_POReset
	  section.data(32).logicalSrcIdx = 80;
	  section.data(32).dtTransOffset = 31;
	
	  ;% System_inverted_P.HILInitialize_OOReset
	  section.data(33).logicalSrcIdx = 81;
	  section.data(33).dtTransOffset = 32;
	
	  ;% System_inverted_P.HILInitialize_DOFinal
	  section.data(34).logicalSrcIdx = 82;
	  section.data(34).dtTransOffset = 33;
	
	  ;% System_inverted_P.HILInitialize_DOInitial
	  section.data(35).logicalSrcIdx = 83;
	  section.data(35).dtTransOffset = 34;
	
	  ;% System_inverted_P.HILReadEncoderTimebase_Active
	  section.data(36).logicalSrcIdx = 84;
	  section.data(36).dtTransOffset = 35;
	
	  ;% System_inverted_P.HILWriteAnalog_Active
	  section.data(37).logicalSrcIdx = 85;
	  section.data(37).dtTransOffset = 36;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(7) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (System_inverted_B)
    ;%
      section.nData     = 17;
      section.data(17)  = dumData; %prealloc
      
	  ;% System_inverted_B.HILReadEncoderTimebase_o1
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_B.HILReadEncoderTimebase_o2
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% System_inverted_B.Pendulumcountstorad
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% System_inverted_B.Armcountstorad
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% System_inverted_B.WhiteNoise
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 4;
	
	  ;% System_inverted_B.Output
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 5;
	
	  ;% System_inverted_B.Gain3
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 6;
	
	  ;% System_inverted_B.Sum3
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 7;
	
	  ;% System_inverted_B.Internal
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 8;
	
	  ;% System_inverted_B.Internal_h
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 9;
	
	  ;% System_inverted_B.Sum
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 10;
	
	  ;% System_inverted_B.Gain2
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 14;
	
	  ;% System_inverted_B.Sum1
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 15;
	
	  ;% System_inverted_B.Saturation
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 16;
	
	  ;% System_inverted_B.ForveCCW
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 17;
	
	  ;% System_inverted_B.Sum2
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 18;
	
	  ;% System_inverted_B.TmpSignalConversionAtToWorkspac
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 19;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 6;
    sectIdxOffset = 1;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (System_inverted_DW)
    ;%
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% System_inverted_DW.HILInitialize_FilterFrequency
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_DW.NextOutput
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% System_inverted_DW.HILInitialize_Card
	  section.data(1).logicalSrcIdx = 2;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% System_inverted_DW.HILReadEncoderTimebase_Task
	  section.data(1).logicalSrcIdx = 3;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% System_inverted_DW.HILWriteAnalog_PWORK
	  section.data(1).logicalSrcIdx = 4;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_DW.ToWorkspace_PWORK.LoggedData
	  section.data(2).logicalSrcIdx = 5;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% System_inverted_DW.HILInitialize_ClockModes
	  section.data(1).logicalSrcIdx = 6;
	  section.data(1).dtTransOffset = 0;
	
	  ;% System_inverted_DW.HILInitialize_DOStates
	  section.data(2).logicalSrcIdx = 7;
	  section.data(2).dtTransOffset = 1;
	
	  ;% System_inverted_DW.HILInitialize_QuadratureModes
	  section.data(3).logicalSrcIdx = 8;
	  section.data(3).dtTransOffset = 2;
	
	  ;% System_inverted_DW.HILInitialize_InitialEICounts
	  section.data(4).logicalSrcIdx = 9;
	  section.data(4).dtTransOffset = 4;
	
	  ;% System_inverted_DW.HILReadEncoderTimebase_Buffer
	  section.data(5).logicalSrcIdx = 10;
	  section.data(5).dtTransOffset = 6;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% System_inverted_DW.RandSeed
	  section.data(1).logicalSrcIdx = 11;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(6) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 1737082375;
  targMap.checksum1 = 3670918618;
  targMap.checksum2 = 380511293;
  targMap.checksum3 = 4067965611;

