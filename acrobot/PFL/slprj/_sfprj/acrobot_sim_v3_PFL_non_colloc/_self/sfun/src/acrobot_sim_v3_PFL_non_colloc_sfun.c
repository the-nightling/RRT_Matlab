/* Include files */

#include "acrobot_sim_v3_PFL_non_colloc_sfun.h"
#include "acrobot_sim_v3_PFL_non_colloc_sfun_debug_macros.h"
#include "c1_acrobot_sim_v3_PFL_non_colloc.h"
#include "c2_acrobot_sim_v3_PFL_non_colloc.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _acrobot_sim_v3_PFL_non_collocMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void acrobot_sim_v3_PFL_non_colloc_initializer(void)
{
}

void acrobot_sim_v3_PFL_non_colloc_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_acrobot_sim_v3_PFL_non_colloc_method_dispatcher(SimStruct
  *simstructPtr, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (chartFileNumber==1) {
    c1_acrobot_sim_v3_PFL_non_colloc_method_dispatcher(simstructPtr, method,
      data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_acrobot_sim_v3_PFL_non_colloc_method_dispatcher(simstructPtr, method,
      data);
    return 1;
  }

  return 0;
}

unsigned int sf_acrobot_sim_v3_PFL_non_colloc_process_check_sum_call( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1735657747U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3646656070U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3315564635U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(264597127U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1779151836U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2357336203U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2771271304U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3752573119U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_acrobot_sim_v3_PFL_non_colloc_get_check_sum(mxArray *
            plhs[]);
          sf_c1_acrobot_sim_v3_PFL_non_colloc_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_acrobot_sim_v3_PFL_non_colloc_get_check_sum(mxArray *
            plhs[]);
          sf_c2_acrobot_sim_v3_PFL_non_colloc_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1764838350U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3410240878U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(118138738U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(243351119U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1167556715U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3314835061U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1429057908U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(27858990U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_acrobot_sim_v3_PFL_non_colloc_autoinheritance_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "5RZM27zgbEWVLqL4DMugCC") == 0) {
          extern mxArray
            *sf_c1_acrobot_sim_v3_PFL_non_colloc_get_autoinheritance_info(void);
          plhs[0] = sf_c1_acrobot_sim_v3_PFL_non_colloc_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "Qvuquf4TQKyyolny8PAZ8C") == 0) {
          extern mxArray
            *sf_c2_acrobot_sim_v3_PFL_non_colloc_get_autoinheritance_info(void);
          plhs[0] = sf_c2_acrobot_sim_v3_PFL_non_colloc_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_acrobot_sim_v3_PFL_non_colloc_get_eml_resolved_functions_info
  ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_acrobot_sim_v3_PFL_non_colloc_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_acrobot_sim_v3_PFL_non_colloc_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_acrobot_sim_v3_PFL_non_colloc_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_acrobot_sim_v3_PFL_non_colloc_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_acrobot_sim_v3_PFL_non_colloc_third_party_uses_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "7UdAQBinVxxrLN9orOirDE") == 0) {
          extern mxArray
            *sf_c1_acrobot_sim_v3_PFL_non_colloc_third_party_uses_info(void);
          plhs[0] = sf_c1_acrobot_sim_v3_PFL_non_colloc_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "kA0evLGutThEzPnVk1crCG") == 0) {
          extern mxArray
            *sf_c2_acrobot_sim_v3_PFL_non_colloc_third_party_uses_info(void);
          plhs[0] = sf_c2_acrobot_sim_v3_PFL_non_colloc_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void acrobot_sim_v3_PFL_non_colloc_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _acrobot_sim_v3_PFL_non_collocMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"acrobot_sim_v3_PFL_non_colloc","sfun",0,2,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _acrobot_sim_v3_PFL_non_collocMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _acrobot_sim_v3_PFL_non_collocMachineNumber_,0);
}

void acrobot_sim_v3_PFL_non_colloc_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_acrobot_sim_v3_PFL_non_colloc_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "acrobot_sim_v3_PFL_non_colloc", "acrobot_sim_v3_PFL_non_colloc");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_acrobot_sim_v3_PFL_non_colloc_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
