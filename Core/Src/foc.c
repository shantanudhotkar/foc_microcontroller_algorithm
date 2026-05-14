/*
 * foc.c
 *
 *  Created on: May 13, 2026
 *      Author: Shantanu
 */


/*
 * foc.c
 *
 *  Created on: May 12, 2026
 *      Author: Shantanu
 *
 *  @brief  Field-Oriented Control (FOC) — full pipeline implementation
 *
 *  CHANGES vs previous version:
 *    1. clarke() takes ia, ib only; ic = -(ia+ib) computed here.
 *    2. FOC_Step() takes theta as a direct parameter (no internal atan2).
 *    3. FOC_EstimateTheta() kept as an optional utility — not called by FOC_Step.
 */

#include "foc.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  INTERNAL — PI helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

static void pi_init(FOC_PI_t *pi,
                    float Kp, float Ki, float b,
                    float out_min, float out_max)
{
    pi->Kp      = Kp;
    pi->Ki      = Ki;
    pi->b          = b;
    pi->out_min    = out_min;
    pi->out_max    = out_max;
    pi->integral   = 0.0f;
    pi->prev_error = 0.0f;
}

static float pi_update(FOC_PI_t *pi,
                       float reference, float measured, float dt)
{
	float error  = reference - measured;

	/* Proportional */
	float P_term = pi->Kp * error;

	/* Integral accumulate */
	pi->integral += error * dt;

	/* Anti-windup */
	float i_max = (pi->Ki > 0.0f) ? (pi->out_max - pi->b) / pi->Ki : 1e10f;
	float i_min = (pi->Ki > 0.0f) ? (pi->out_min - pi->b) / pi->Ki : -1e10f;
	if (pi->integral > i_max) pi->integral = i_max;
	if (pi->integral < i_min) pi->integral = i_min;

	/* Integral term */
	float I_term = pi->Ki * pi->integral;

	/* Output with bias */
	float output = pi->b + P_term + I_term;

	/* Hard output clamp */
	if (output > pi->out_max) output = pi->out_max;
	if (output < pi->out_min) output = pi->out_min;

	pi->prev_error = error;
	return output;
}

static void pi_reset(FOC_PI_t *pi)
{
    pi->integral   = 0.0f;
    pi->prev_error = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  INTERNAL — Clarke Transform  (2-phase input; ic derived)
 *
 *  ic is NOT a separate input — it is always -(ia + ib) for balanced 3-phase.
 *  Substituting ic = -(ia+ib) into the full amplitude-invariant form gives:
 *
 *    i_alpha = ia
 *    i_beta  = (ia + 2·ib) / √3
 *
 *  This is mathematically identical to the 3-input form with balanced phases.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void clarke(float ia, float ib,
                   FOC_AlphaBeta_t *out)
{
    /* ic derived here — caller does NOT need to supply it */
    /* float ic = -(ia + ib);  (substituted analytically below) */

    out->alpha = ia;                                   /* simplifies to ia  */
    out->beta  = (ia + 2.0f * ib) * FOC_ONE_OVER_SQRT3; /* (ia+2·ib)/√3     */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  INTERNAL — Park Transform
 *
 *  id =  alpha·cos(θ) + beta·sin(θ)
 *  iq = -alpha·sin(θ) + beta·cos(θ)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void park(const FOC_AlphaBeta_t *ab, float theta,
                 FOC_DQ_t *out)
{
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    out->d =  ab->alpha * cos_t + ab->beta * sin_t;
    out->q = -ab->alpha * sin_t + ab->beta * cos_t;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  INTERNAL — Inverse Park Transform
 *
 *  v_alpha = vd·cos(θ) - vq·sin(θ)
 *  v_beta  = vd·sin(θ) + vq·cos(θ)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void inv_park(float vd, float vq, float theta,
                     FOC_AlphaBeta_t *out)
{
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    out->alpha = vd * cos_t - vq * sin_t;
    out->beta  = vd * sin_t + vq * cos_t;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  INTERNAL — Inverse Clarke Transform
 *
 *  va =  alpha
 *  vb = -alpha/2 + beta·(√3/2)
 *  vc = -alpha/2 - beta·(√3/2)
 *
 *  Self-check: va + vb + vc = 0  ✓
 * ═══════════════════════════════════════════════════════════════════════════ */
static void inv_clarke(const FOC_AlphaBeta_t *ab,
                       FOC_ThreePhase_t *out)
{
    out->a =  ab->alpha;
    out->b = -(FOC_ONE_HALF * ab->alpha) + (FOC_SQRT3_OVER_2 * ab->beta);
    out->c = -(FOC_ONE_HALF * ab->alpha) - (FOC_SQRT3_OVER_2 * ab->beta);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC — FOC_EstimateTheta  (optional utility, NOT used by FOC_Step)
 *
 *  Returns atan2(beta, alpha) — the angle of the current vector.
 *
 *  DO NOT feed this back into FOC_Step as theta.  Doing so collapses the
 *  Park transform: id_meas = |i| (amplitude), iq_meas = 0 — always.
 *  The PI_q then regulates against a permanently-zero iq, which is useless.
 *
 *  Use this only for:
 *    • Offline phase-angle inspection
 *    • Sensorless startup before observer lock
 * ═══════════════════════════════════════════════════════════════════════════ */
float FOC_EstimateTheta(float alpha, float beta)
{
    return atan2f(beta, alpha);   /* returns -π to +π */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC — FOC_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void FOC_Init(FOC_Handle_t *hfoc,
              float id_ref, float iq_ref,
              float Kp, float Ki,
              float out_min, float out_max)
{
    hfoc->id_ref = id_ref;
    hfoc->iq_ref = iq_ref;
    hfoc->theta  = 0.0f;
    hfoc->omega  = 0.0f;

    /* bias = 0 for standard PI */
    pi_init(&hfoc->pi_d, Kp, Ki, 0.0f, out_min, out_max);
    pi_init(&hfoc->pi_q, Kp, Ki, 0.0f, out_min, out_max);

    /* Clear intermediate buffers */
    hfoc->clarke_out     = (FOC_AlphaBeta_t){0};
    hfoc->park_out       = (FOC_DQ_t){0};
    hfoc->pi_out         = (FOC_DQ_t){0};
    hfoc->inv_park_out   = (FOC_AlphaBeta_t){0};
    hfoc->inv_clarke_out = (FOC_ThreePhase_t){0};
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC — FOC_Step
 *
 *  One complete FOC iteration.  Call every dt seconds.
 *
 *  ia, ib  — 2-phase measured currents (ic computed internally)
 *  theta   — electrical angle from encoder or synth_theta (NOT atan2)
 *  dt      — control period in seconds
 *
 *  Signal chain:
 *    ia, ib
 *      │  ic = -(ia+ib) internally
 *      ▼  [1] Clarke
 *    i_alpha, i_beta
 *      ▼  [2] Park(theta)
 *    id_meas, iq_meas
 *      ▼  [3] PI_d(id_ref, id_meas)   PI_q(iq_ref, iq_meas)
 *    vd, vq
 *      ▼  [4] InvPark(theta)
 *    v_alpha, v_beta
 *      ▼  [5] InvClarke
 *    va, vb, vc  →  DAC / SVPWM
 * ═══════════════════════════════════════════════════════════════════════════ */
void FOC_Step(FOC_Handle_t *hfoc,
              float ia, float ib,
              float theta, float dt)
{
    /* Store theta */
    hfoc->theta = theta;

    /* ── [1] Clarke: ia, ib → αβ  (ic = -(ia+ib) internally) ───────── */
    clarke(ia, ib, &hfoc->clarke_out);

    /* ── [2] Park: αβ, θ → dq ──────────────────────────────────────── */
    park(&hfoc->clarke_out, hfoc->theta, &hfoc->park_out);

    /* ── [3] PI controllers ─────────────────────────────────────────── */
    hfoc->pi_out.d = pi_update(&hfoc->pi_d,
                                hfoc->id_ref,
                                hfoc->park_out.d,
                                dt);

    hfoc->pi_out.q = pi_update(&hfoc->pi_q,
                                hfoc->iq_ref,
                                hfoc->park_out.q,
                                dt);

    /* ── [4] Inverse Park: vd, vq, θ → αβ voltages ─────────────────── */
    inv_park(hfoc->pi_out.d, hfoc->pi_out.q,
             hfoc->theta,
             &hfoc->inv_park_out);

    /* ── [5] Inverse Clarke: αβ → va, vb, vc ───────────────────────── */
    inv_clarke(&hfoc->inv_park_out, &hfoc->inv_clarke_out);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC — FOC_Reset
 * ═══════════════════════════════════════════════════════════════════════════ */
void FOC_Reset(FOC_Handle_t *hfoc)
{
    pi_reset(&hfoc->pi_d);
    pi_reset(&hfoc->pi_q);
    hfoc->theta = 0.0f;
    hfoc->omega = 0.0f;
}
