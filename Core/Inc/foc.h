/*
 * foc.h
 *
 *  Created on: May 13, 2026
 *      Author: Shantanu
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

/*
 * foc.h
 *
 *  Created on: May 12, 2026
 *      Author: Shantanu
 *
 *  @brief  Field-Oriented Control (FOC) pipeline — single include
 *
 *  Full signal chain:
 *
 *    [ia, ib]   (ic derived internally as -(ia+ib))
 *         │
 *         ▼
 *    Clarke_Transform()      →  i_alpha, i_beta   (stationary αβ frame)
 *         │
 *         ▼
 *    Park_Transform(θ)       →  id, iq             (rotating dq frame)
 *         │
 *         ▼
 *    PI_Update (d & q)       →  vd, vq             (voltage demands)
 *         │
 *         ▼
 *    InvPark_Transform(θ)    →  v_alpha, v_beta    (back to stationary frame)
 *         │
 *         ▼
 *    InvClarke_Transform()   →  va, vb, vc         (3-phase voltage outputs)
 *         │
 *         ▼
 *    DAC / PWM modulator
 *
 *  CHANGES vs previous version:
 *    1. Clarke now takes only ia, ib — ic = -(ia+ib) computed internally.
 *    2. FOC_Step takes theta as a parameter (from encoder / synth angle).
 *       estimate_theta() still available as FOC_EstimateTheta() if needed.
 *    3. PI output limits now derived from DAC range (±2047), not arbitrary.
 */

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  SECTION 1 — Clarke / Park constants
 * ═══════════════════════════════════════════════════════════════════════════ */
#define FOC_ONE_THIRD        0.33333f
#define FOC_TWO_THIRDS       0.66667f
#define FOC_ONE_OVER_SQRT3   0.57735f   /* 1 / √3                   */
#define FOC_SQRT3_OVER_2     0.86603f   /* √3 / 2                   */
#define FOC_ONE_HALF         0.50000f   /* 1 / 2                    */

/* ═══════════════════════════════════════════════════════════════════════════
 *  SECTION 2 — Result structures
 * ═══════════════════════════════════════════════════════════════════════════ */

/** αβ stationary-frame currents (output of Clarke, input to Park) */
typedef struct
{
    float alpha;
    float beta;
} FOC_AlphaBeta_t;

/** dq rotating-frame currents (output of Park, input to PI) */
typedef struct
{
    float d;    /* flux-producing   component */
    float q;    /* torque-producing component */
} FOC_DQ_t;

/** 3-phase reconstructed voltages (output of Inv-Clarke → DAC/PWM) */
typedef struct
{
    float a;
    float b;
    float c;
} FOC_ThreePhase_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  SECTION 3 — PI Controller
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct
{
	float Kp;
	float Ki;          /* integral time constant               */
    float b;            /* bias / feedforward offset            */
    float out_min;      /* output lower rail                    */
    float out_max;      /* output upper rail                    */
    float integral;     /* running integral accumulator         */
    float prev_error;   /* stored for derivative (unused in PI) */
} FOC_PI_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  SECTION 4 — Top-level FOC state (one instance per motor)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct
{
    /* --- Angle (electrical, radians) --------------------------------- */
    float theta;            /* current electrical angle             */
    float omega;            /* electrical angular velocity (rad/s)  */

    /* --- PI controllers --------------------------------------------- */
    FOC_PI_t pi_d;          /* d-axis (flux) controller             */
    FOC_PI_t pi_q;          /* q-axis (torque) controller           */

    /* --- References -------------------------------------------------- */
    float id_ref;           /* d-axis current reference             */
    float iq_ref;           /* q-axis current reference             */

    /* --- Intermediate signals (kept for logging / debug) ------------- */
    FOC_AlphaBeta_t  clarke_out;      /* i_alpha, i_beta              */
    FOC_DQ_t         park_out;        /* id_meas, iq_meas             */
    FOC_DQ_t         pi_out;          /* vd, vq                       */
    FOC_AlphaBeta_t  inv_park_out;    /* v_alpha, v_beta              */
    FOC_ThreePhase_t inv_clarke_out;  /* va, vb, vc                   */
} FOC_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  SECTION 5 — Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise the FOC handle and both PI controllers.
 *
 * @param  hfoc      Pointer to FOC_Handle_t
 * @param  id_ref    d-axis current reference  (0.0f for PMSM, SPMSM)
 * @param  iq_ref    q-axis current reference  (torque demand, units = amps)
 * @param  Kc        PI proportional gain
 * @param  Ti        PI integral time constant  (seconds)
 * @param  out_min   PI output lower rail  — set to -(DAC_MIDSCALE - 1) = -2047
 * @param  out_max   PI output upper rail  — set to  (DAC_MIDSCALE - 1) = +2047
 */
void FOC_Init(FOC_Handle_t *hfoc,
              float id_ref, float iq_ref,
              float Kp, float Ki,
              float out_min, float out_max);
/**
 * @brief  Run one FOC step — full pipeline from ia/ib to 3-phase voltages.
 *
 *  Inputs:
 *    ia, ib   — measured (or synthetic) phase currents.
 *               ic is derived internally: ic = -(ia + ib).
 *    theta    — electrical angle in radians.  Provide from:
 *                 • encoder/resolver  (real motor), OR
 *                 • synth_theta       (open-loop demo).
 *               Do NOT use atan2(beta,alpha) for theta — that locks
 *               id_meas to the current amplitude and iq_meas to zero.
 *    dt       — control loop period in seconds (e.g. 0.001)
 *
 *  Steps executed:
 *    1. ic = -(ia + ib)
 *    2. Clarke  (ia,ib,ic → αβ)
 *    3. Park    (αβ, θ   → dq)
 *    4. PI_d, PI_q
 *    5. Inv-Park  (vd,vq,θ → αβ voltages)
 *    6. Inv-Clarke (αβ    → va,vb,vc)
 *
 *  Results stored in hfoc->inv_clarke_out.{a,b,c} → feed to DAC/PWM.
 *
 * @param  hfoc   Pointer to FOC_Handle_t
 * @param  ia     Phase-A measured current
 * @param  ib     Phase-B measured current
 * @param  theta  Electrical angle (radians) — from encoder or synth
 * @param  dt     Control loop period (seconds)
 */
void FOC_Step(FOC_Handle_t *hfoc,
              float ia, float ib,
              float theta, float dt);

/**
 * @brief  Reset both PI integrators and the angle accumulator.
 * @param  hfoc  Pointer to FOC_Handle_t
 */
void FOC_Reset(FOC_Handle_t *hfoc);

/**
 * @brief  Optional — estimate theta from αβ current vector via atan2.
 *
 *  WARNING: Using this as the Park angle causes id_meas = current amplitude,
 *  iq_meas = 0, always — regardless of motor state. Only use for
 *  offline angle-detection checks, NOT as the FOC_Step theta input.
 *
 * @param  alpha  Clarke output alpha
 * @param  beta   Clarke output beta
 * @return Estimated angle in radians (-π to +π)
 */
float FOC_EstimateTheta(float alpha, float beta);

#ifdef __cplusplus
}
#endif



#endif /* INC_FOC_H_ */
