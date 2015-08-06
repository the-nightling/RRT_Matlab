/* Include files */

#include <stddef.h>
#include "blas.h"
#include "acrobot_sim_v3_sfun.h"
#include "c2_acrobot_sim_v3.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "acrobot_sim_v3_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[9] = { "nargin", "nargout", "theta1",
  "theta1_d", "theta2", "theta2_d", "tau", "theta1_dd", "theta2_dd" };

/* Function Declarations */
static void initialize_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void initialize_params_c2_acrobot_sim_v3
  (SFc2_acrobot_sim_v3InstanceStruct *chartInstance);
static void enable_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void disable_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void c2_update_debugger_state_c2_acrobot_sim_v3
  (SFc2_acrobot_sim_v3InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_acrobot_sim_v3
  (SFc2_acrobot_sim_v3InstanceStruct *chartInstance);
static void set_sim_state_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void sf_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void initSimStructsc2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void registerMessagesc2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static real_T c2_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_theta1_dd, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_c_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_d_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_acrobot_sim_v3, const char_T *
  c2_identifier);
static uint8_T c2_e_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_acrobot_sim_v3 = 0U;
}

static void initialize_params_c2_acrobot_sim_v3
  (SFc2_acrobot_sim_v3InstanceStruct *chartInstance)
{
}

static void enable_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_acrobot_sim_v3
  (SFc2_acrobot_sim_v3InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_acrobot_sim_v3
  (SFc2_acrobot_sim_v3InstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  uint8_T c2_c_hoistedGlobal;
  uint8_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T *c2_theta1_dd;
  real_T *c2_theta2_dd;
  c2_theta2_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_theta1_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(3), FALSE);
  c2_hoistedGlobal = *c2_theta1_dd;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_theta2_dd;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = chartInstance->c2_is_active_c2_acrobot_sim_v3;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T *c2_theta1_dd;
  real_T *c2_theta2_dd;
  c2_theta2_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_theta1_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  *c2_theta1_dd = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 0)), "theta1_dd");
  *c2_theta2_dd = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 1)), "theta2_dd");
  chartInstance->c2_is_active_c2_acrobot_sim_v3 = c2_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
     "is_active_c2_acrobot_sim_v3");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_acrobot_sim_v3(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
}

static void sf_c2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_e_hoistedGlobal;
  real_T c2_theta1;
  real_T c2_theta1_d;
  real_T c2_theta2;
  real_T c2_theta2_d;
  real_T c2_tau;
  uint32_T c2_debug_family_var_map[9];
  real_T c2_nargin = 5.0;
  real_T c2_nargout = 2.0;
  real_T c2_theta1_dd;
  real_T c2_theta2_dd;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_e_y = NULL;
  const mxArray *c2_b_theta2_dd = NULL;
  const mxArray *c2_b_theta1_dd = NULL;
  real_T *c2_b_theta1;
  real_T *c2_c_theta1_dd;
  real_T *c2_b_theta1_d;
  real_T *c2_b_theta2;
  real_T *c2_b_theta2_d;
  real_T *c2_c_theta2_dd;
  real_T *c2_b_tau;
  c2_b_tau = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_c_theta2_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_theta2_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_theta2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_theta1_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_c_theta1_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_theta1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_b_theta1, 0U);
  _SFD_DATA_RANGE_CHECK(*c2_c_theta1_dd, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_b_theta1_d, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_b_theta2, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_b_theta2_d, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_c_theta2_dd, 5U);
  _SFD_DATA_RANGE_CHECK(*c2_b_tau, 6U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_theta1;
  c2_b_hoistedGlobal = *c2_b_theta1_d;
  c2_c_hoistedGlobal = *c2_b_theta2;
  c2_d_hoistedGlobal = *c2_b_theta2_d;
  c2_e_hoistedGlobal = *c2_b_tau;
  c2_theta1 = c2_hoistedGlobal;
  c2_theta1_d = c2_b_hoistedGlobal;
  c2_theta2 = c2_c_hoistedGlobal;
  c2_theta2_d = c2_d_hoistedGlobal;
  c2_tau = c2_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_theta1, 2U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_theta1_d, 3U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_theta2, 4U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_theta2_d, 5U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_tau, 6U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta1_dd, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta2_dd, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_theta1_dd = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_theta2_dd = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_u = c2_theta1;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  c2_b_u = c2_theta1_d;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  c2_c_u = c2_theta2;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  c2_d_u = c2_theta2_d;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  c2_e_u = c2_tau;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_call_debug("AcrobotDynamics", 2U, 5U, 14, c2_y, 14, c2_b_y, 14, c2_c_y,
                    14, c2_d_y, 14, c2_e_y, &c2_b_theta1_dd, &c2_b_theta2_dd);
  c2_theta1_dd = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_theta1_dd),
    "theta1_dd");
  c2_theta2_dd = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_theta2_dd),
    "theta2_dd");
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  sf_mex_destroy(&c2_b_theta1_dd);
  sf_mex_destroy(&c2_b_theta2_dd);
  *c2_c_theta1_dd = c2_theta1_dd;
  *c2_c_theta2_dd = c2_theta2_dd;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_acrobot_sim_v3MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
}

static void registerMessagesc2_acrobot_sim_v3(SFc2_acrobot_sim_v3InstanceStruct *
  chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc2_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_theta1_dd;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc2_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c2_theta1_dd = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_theta1_dd),
    &c2_thisId);
  sf_mex_destroy(&c2_theta1_dd);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_acrobot_sim_v3_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), FALSE);
  return c2_nameCaptureInfo;
}

static real_T c2_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_theta1_dd, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_theta1_dd),
    &c2_thisId);
  sf_mex_destroy(&c2_theta1_dd);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc2_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_c_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i0, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc2_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_d_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_acrobot_sim_v3, const char_T *
  c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_acrobot_sim_v3), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_acrobot_sim_v3);
  return c2_y;
}

static uint8_T c2_e_emlrt_marshallIn(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_acrobot_sim_v3_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(474496428U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(630179175U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4249953308U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(179988483U);
}

mxArray *sf_c2_acrobot_sim_v3_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("Qvuquf4TQKyyolny8PAZ8C");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_acrobot_sim_v3_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c2_acrobot_sim_v3(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"theta1_dd\",},{M[1],M[9],T\"theta2_dd\",},{M[8],M[0],T\"is_active_c2_acrobot_sim_v3\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_acrobot_sim_v3_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_acrobot_sim_v3InstanceStruct *chartInstance;
    chartInstance = (SFc2_acrobot_sim_v3InstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _acrobot_sim_v3MachineNumber_,
           2,
           1,
           1,
           7,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_acrobot_sim_v3MachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_acrobot_sim_v3MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _acrobot_sim_v3MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"theta1");
          _SFD_SET_DATA_PROPS(1,2,0,1,"theta1_dd");
          _SFD_SET_DATA_PROPS(2,1,1,0,"theta1_d");
          _SFD_SET_DATA_PROPS(3,1,1,0,"theta2");
          _SFD_SET_DATA_PROPS(4,1,1,0,"theta2_d");
          _SFD_SET_DATA_PROPS(5,2,0,1,"theta2_dd");
          _SFD_SET_DATA_PROPS(6,1,1,0,"tau");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,267);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c2_theta1;
          real_T *c2_theta1_dd;
          real_T *c2_theta1_d;
          real_T *c2_theta2;
          real_T *c2_theta2_d;
          real_T *c2_theta2_dd;
          real_T *c2_tau;
          c2_tau = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c2_theta2_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_theta2_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_theta2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_theta1_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_theta1_dd = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_theta1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_theta1);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_theta1_dd);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_theta1_d);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_theta2);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_theta2_d);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_theta2_dd);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_tau);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _acrobot_sim_v3MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "kA0evLGutThEzPnVk1crCG";
}

static void sf_opaque_initialize_c2_acrobot_sim_v3(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
  initialize_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_acrobot_sim_v3(void *chartInstanceVar)
{
  enable_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_acrobot_sim_v3(void *chartInstanceVar)
{
  disable_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_acrobot_sim_v3(void *chartInstanceVar)
{
  sf_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_acrobot_sim_v3(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_acrobot_sim_v3
    ((SFc2_acrobot_sim_v3InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_acrobot_sim_v3();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_acrobot_sim_v3(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_acrobot_sim_v3();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_acrobot_sim_v3(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_acrobot_sim_v3(S);
}

static void sf_opaque_set_sim_state_c2_acrobot_sim_v3(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_acrobot_sim_v3(S, st);
}

static void sf_opaque_terminate_c2_acrobot_sim_v3(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_acrobot_sim_v3InstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_acrobot_sim_v3_optimization_info();
    }

    finalize_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_acrobot_sim_v3(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_acrobot_sim_v3((SFc2_acrobot_sim_v3InstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_acrobot_sim_v3(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_acrobot_sim_v3_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2820354298U));
  ssSetChecksum1(S,(40152271U));
  ssSetChecksum2(S,(1639514904U));
  ssSetChecksum3(S,(958169865U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_acrobot_sim_v3(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_acrobot_sim_v3(SimStruct *S)
{
  SFc2_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc2_acrobot_sim_v3InstanceStruct *)utMalloc(sizeof
    (SFc2_acrobot_sim_v3InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_acrobot_sim_v3InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_acrobot_sim_v3;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_acrobot_sim_v3;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_acrobot_sim_v3;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_acrobot_sim_v3;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_acrobot_sim_v3;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_acrobot_sim_v3;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_acrobot_sim_v3;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_acrobot_sim_v3;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_acrobot_sim_v3;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_acrobot_sim_v3;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_acrobot_sim_v3;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_acrobot_sim_v3_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_acrobot_sim_v3(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_acrobot_sim_v3(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_acrobot_sim_v3(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_acrobot_sim_v3_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
