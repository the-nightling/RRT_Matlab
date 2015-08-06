/* Include files */

#include <stddef.h>
#include "blas.h"
#include "acrobot_sim_v3_sfun.h"
#include "c1_acrobot_sim_v3.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "acrobot_sim_v3_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[8] = { "nargin", "nargout", "theta1",
  "theta1_d", "theta2", "theta2_d", "theta2_dd", "tau" };

/* Function Declarations */
static void initialize_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void initialize_params_c1_acrobot_sim_v3
  (SFc1_acrobot_sim_v3InstanceStruct *chartInstance);
static void enable_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void disable_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void c1_update_debugger_state_c1_acrobot_sim_v3
  (SFc1_acrobot_sim_v3InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_acrobot_sim_v3
  (SFc1_acrobot_sim_v3InstanceStruct *chartInstance);
static void set_sim_state_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void sf_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void initSimStructsc1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance);
static void registerMessagesc1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static real_T c1_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_PFL_colloc, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_c_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_d_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_acrobot_sim_v3, const char_T *
  c1_identifier);
static uint8_T c1_e_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_is_active_c1_acrobot_sim_v3 = 0U;
}

static void initialize_params_c1_acrobot_sim_v3
  (SFc1_acrobot_sim_v3InstanceStruct *chartInstance)
{
}

static void enable_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_acrobot_sim_v3
  (SFc1_acrobot_sim_v3InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c1_acrobot_sim_v3
  (SFc1_acrobot_sim_v3InstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  uint8_T c1_b_hoistedGlobal;
  uint8_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T *c1_tau;
  c1_tau = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(2), FALSE);
  c1_hoistedGlobal = *c1_tau;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = chartInstance->c1_is_active_c1_acrobot_sim_v3;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T *c1_tau;
  c1_tau = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  *c1_tau = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
    "tau");
  chartInstance->c1_is_active_c1_acrobot_sim_v3 = c1_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
     "is_active_c1_acrobot_sim_v3");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_acrobot_sim_v3(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
}

static void sf_c1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_e_hoistedGlobal;
  real_T c1_theta1;
  real_T c1_theta1_d;
  real_T c1_theta2;
  real_T c1_theta2_d;
  real_T c1_theta2_dd;
  uint32_T c1_debug_family_var_map[8];
  real_T c1_nargin = 5.0;
  real_T c1_nargout = 1.0;
  real_T c1_tau;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_c_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_d_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_e_u;
  const mxArray *c1_e_y = NULL;
  real_T *c1_b_theta1;
  real_T *c1_b_theta1_d;
  real_T *c1_b_theta2;
  real_T *c1_b_theta2_d;
  real_T *c1_b_theta2_dd;
  real_T *c1_b_tau;
  c1_b_tau = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_theta2_dd = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c1_b_theta2_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c1_b_theta2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c1_b_theta1_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_theta1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c1_b_theta1, 0U);
  _SFD_DATA_RANGE_CHECK(*c1_b_theta1_d, 1U);
  _SFD_DATA_RANGE_CHECK(*c1_b_theta2, 2U);
  _SFD_DATA_RANGE_CHECK(*c1_b_theta2_d, 3U);
  _SFD_DATA_RANGE_CHECK(*c1_b_theta2_dd, 4U);
  _SFD_DATA_RANGE_CHECK(*c1_b_tau, 5U);
  chartInstance->c1_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *c1_b_theta1;
  c1_b_hoistedGlobal = *c1_b_theta1_d;
  c1_c_hoistedGlobal = *c1_b_theta2;
  c1_d_hoistedGlobal = *c1_b_theta2_d;
  c1_e_hoistedGlobal = *c1_b_theta2_dd;
  c1_theta1 = c1_hoistedGlobal;
  c1_theta1_d = c1_b_hoistedGlobal;
  c1_theta2 = c1_c_hoistedGlobal;
  c1_theta2_d = c1_d_hoistedGlobal;
  c1_theta2_dd = c1_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_theta1, 2U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_theta1_d, 3U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_theta2, 4U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_theta2_d, 5U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_theta2_dd, 6U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_tau, 7U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  c1_tau = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  c1_u = c1_theta1;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  c1_b_u = c1_theta1_d;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  c1_c_u = c1_theta2;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  c1_d_u = c1_theta2_d;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  c1_e_u = c1_theta2_dd;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), FALSE);
  c1_tau = c1_emlrt_marshallIn(chartInstance, sf_mex_call_debug("PFL_colloc", 1U,
    5U, 14, c1_y, 14, c1_b_y, 14, c1_c_y, 14, c1_d_y, 14, c1_e_y), "PFL_colloc");
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
  *c1_b_tau = c1_tau;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_acrobot_sim_v3MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance)
{
}

static void registerMessagesc1_acrobot_sim_v3(SFc1_acrobot_sim_v3InstanceStruct *
  chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc1_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_PFL_colloc;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc1_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c1_PFL_colloc = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_PFL_colloc),
    &c1_thisId);
  sf_mex_destroy(&c1_PFL_colloc);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_acrobot_sim_v3_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), FALSE);
  return c1_nameCaptureInfo;
}

static real_T c1_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_PFL_colloc, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_PFL_colloc),
    &c1_thisId);
  sf_mex_destroy(&c1_PFL_colloc);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc1_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_c_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i0, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc1_acrobot_sim_v3InstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_d_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_acrobot_sim_v3, const char_T *
  c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_acrobot_sim_v3), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_acrobot_sim_v3);
  return c1_y;
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_acrobot_sim_v3InstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_acrobot_sim_v3InstanceStruct
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

void sf_c1_acrobot_sim_v3_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(762252693U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3368649061U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(832830880U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2672946708U);
}

mxArray *sf_c1_acrobot_sim_v3_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("03uYnfjdZRIpGDtY1lhp1E");
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

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_acrobot_sim_v3_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

static const mxArray *sf_get_sim_state_info_c1_acrobot_sim_v3(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[10],T\"tau\",},{M[8],M[0],T\"is_active_c1_acrobot_sim_v3\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_acrobot_sim_v3_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_acrobot_sim_v3InstanceStruct *chartInstance;
    chartInstance = (SFc1_acrobot_sim_v3InstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _acrobot_sim_v3MachineNumber_,
           1,
           1,
           1,
           6,
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
          _SFD_SET_DATA_PROPS(1,1,1,0,"theta1_d");
          _SFD_SET_DATA_PROPS(2,1,1,0,"theta2");
          _SFD_SET_DATA_PROPS(3,1,1,0,"theta2_d");
          _SFD_SET_DATA_PROPS(4,1,1,0,"theta2_dd");
          _SFD_SET_DATA_PROPS(5,2,0,1,"tau");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,204);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);

        {
          real_T *c1_theta1;
          real_T *c1_theta1_d;
          real_T *c1_theta2;
          real_T *c1_theta2_d;
          real_T *c1_theta2_dd;
          real_T *c1_tau;
          c1_tau = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c1_theta2_dd = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c1_theta2_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c1_theta2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c1_theta1_d = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c1_theta1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c1_theta1);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_theta1_d);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_theta2);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_theta2_d);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_theta2_dd);
          _SFD_SET_DATA_VALUE_PTR(5U, c1_tau);
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
  return "iGCi41KI2uN16pFOezYawE";
}

static void sf_opaque_initialize_c1_acrobot_sim_v3(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
  initialize_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_acrobot_sim_v3(void *chartInstanceVar)
{
  enable_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_acrobot_sim_v3(void *chartInstanceVar)
{
  disable_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_acrobot_sim_v3(void *chartInstanceVar)
{
  sf_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_acrobot_sim_v3(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_acrobot_sim_v3
    ((SFc1_acrobot_sim_v3InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_acrobot_sim_v3();/* state var info */
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

extern void sf_internal_set_sim_state_c1_acrobot_sim_v3(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_acrobot_sim_v3();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_acrobot_sim_v3(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_acrobot_sim_v3(S);
}

static void sf_opaque_set_sim_state_c1_acrobot_sim_v3(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c1_acrobot_sim_v3(S, st);
}

static void sf_opaque_terminate_c1_acrobot_sim_v3(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_acrobot_sim_v3InstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_acrobot_sim_v3_optimization_info();
    }

    finalize_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_acrobot_sim_v3(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_acrobot_sim_v3((SFc1_acrobot_sim_v3InstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_acrobot_sim_v3(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_acrobot_sim_v3_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2127883473U));
  ssSetChecksum1(S,(540082020U));
  ssSetChecksum2(S,(2832901741U));
  ssSetChecksum3(S,(1189626968U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_acrobot_sim_v3(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_acrobot_sim_v3(SimStruct *S)
{
  SFc1_acrobot_sim_v3InstanceStruct *chartInstance;
  chartInstance = (SFc1_acrobot_sim_v3InstanceStruct *)utMalloc(sizeof
    (SFc1_acrobot_sim_v3InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_acrobot_sim_v3InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_acrobot_sim_v3;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_acrobot_sim_v3;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_acrobot_sim_v3;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_acrobot_sim_v3;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_acrobot_sim_v3;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_acrobot_sim_v3;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_acrobot_sim_v3;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_acrobot_sim_v3;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_acrobot_sim_v3;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_acrobot_sim_v3;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_acrobot_sim_v3;
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

void c1_acrobot_sim_v3_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_acrobot_sim_v3(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_acrobot_sim_v3(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_acrobot_sim_v3(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_acrobot_sim_v3_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
