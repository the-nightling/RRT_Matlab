#ifndef __c1_acrobot_sim_v3_PFL_colloc_h__
#define __c1_acrobot_sim_v3_PFL_colloc_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_SFc1_acrobot_sim_v3_PFL_collocInstanceStruct
#define typedef_SFc1_acrobot_sim_v3_PFL_collocInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_acrobot_sim_v3_PFL_colloc;
} SFc1_acrobot_sim_v3_PFL_collocInstanceStruct;

#endif                                 /*typedef_SFc1_acrobot_sim_v3_PFL_collocInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_acrobot_sim_v3_PFL_colloc_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_acrobot_sim_v3_PFL_colloc_get_check_sum(mxArray *plhs[]);
extern void c1_acrobot_sim_v3_PFL_colloc_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
