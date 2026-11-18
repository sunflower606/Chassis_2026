#include "speed_loop.h"

#include <stddef.h>

#include "bsp_encoder.h"
#include "chassis_config.h"
#include "Delay.h"

/* 名义上的 PI 周期（秒）= 分频 × 控制周期。
 * 5 ms 就走一次 PI 的话，编码器每拍只走十几个计数，
 * 1 个计数 ≈ 几十‰ 速度，PI 会跟着量化噪声来回抖，所以攒 4 拍算一次。
 *
 * ⚠️ 这只是「名义值」，不能拿它当真实 dt 用：主循环里还有 OLED 整屏刷新
 *    （每次阻塞约 10~20 ms）、串口 sprintf 等，实际窗口会在 20~45 ms 之间漂。
 *    真 dt 由 Delay_GetMs() 实测，见 SpeedLoop_Update()。 */
#define SPEED_LOOP_NOMINAL_DT_S  (((float)CHASSIS_SPEED_LOOP_DIV * (float)CHASSIS_CONTROL_PERIOD_MS) / 1000.0f)

/* 一个计数对应多少 ‰ 速度（按**实测** dt 现算）：
 *   CNT_FULL_PER_SEC 是「占空比 1000‰ 时每秒的计数」（标定值），
 *   一个窗口（dt 秒）内走满 CNT_FULL_PER_SEC × dt 个计数 = 1000‰。 */
#define SPEED_PER_COUNT_AT(dt_s)  (1000.0f / (CHASSIS_SPEED_CNT_FULL_PER_SEC * (dt_s)))

typedef struct {
    float integ;   /* 积分项，单位：‰ 占空比 */
    float meas;    /* 实测速度，单位：‰（正 = 后退） */
    float duty;    /* 上一次算出的占空比，单位：‰ */
} SpeedLoopWheel_t;

static SpeedLoopWheel_t s_left;
static SpeedLoopWheel_t s_right;

/* 编码器增量累加器：两个 PI 窗口之间先攒着 */
static int32_t s_acc_left = 0;
static int32_t s_acc_right = 0;
static uint8_t s_div_count = 0U;

/* 上一个 PI 窗口开始的时刻（ms）。用来算真实 dt，
 * 避免把 OLED 刷屏造成的 45 ms 窗口当成 20 ms，导致测速偏高、占空比被误砍。 */
static uint32_t s_last_pi_ms = 0U;

static float SpeedLoop_ClampF(float value, float low, float high)
{
    if (value > high) {
        return high;
    }
    if (value < low) {
        return low;
    }

    return value;
}

static int16_t SpeedLoop_ToI16(float value)
{
    /* 四舍五入，负数对称处理（直接强转是截断，会让负方向偏小） */
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/* 车轮模型：目标速度 -> 占空比
 * dt_s 为本窗口的真实时长（秒），积分与速率限制都按它缩放。 */
static void SpeedLoop_Step(SpeedLoopWheel_t *wheel, float target, float dt_s)
{
    const float duty_max = (float)CHASSIS_MOTOR_DUTY_MAX;
    const float prev_duty = wheel->duty;   /* 上一窗口的输出，用于速率限制 */
    float error;

    /* 目标为 0：不刹车，直接断电滑行；积分清零，避免下次起步带旧值 */
    if (target == 0.0f) {
        wheel->duty = 0.0f;
        wheel->integ = 0.0f;
        return;
    }

    error = target - wheel->meas;

    /* 积分：单位 ‰/(‰·s) × ‰ × s = ‰ */
    wheel->integ += CHASSIS_SPEED_KI * error * dt_s;
    wheel->integ = SpeedLoop_ClampF(wheel->integ,
                                    -CHASSIS_SPEED_INTEG_MAX,
                                    CHASSIS_SPEED_INTEG_MAX);

    /* 前馈 + 比例 + 积分。
     * 前馈直接取 target：按标定，稳态下「speed ‰」正好等于「duty ‰」，
     * 所以一开始就给一个大致正确的占空比，PI 只负责修正剩余误差
     * （这样即使 KI 很小也不会出现「起步半天不动」）。 */
    wheel->duty = target + CHASSIS_SPEED_KP * error + wheel->integ;

    /* 抗积分饱和：输出被夹住、且误差还在把输出往同方向推，就把这一拍的积分撤掉 */
    if (wheel->duty > duty_max) {
        wheel->duty = duty_max;
        if (error > 0.0f) {
            wheel->integ -= CHASSIS_SPEED_KI * error * dt_s;
        }
    } else if (wheel->duty < -duty_max) {
        wheel->duty = -duty_max;
        if (error < 0.0f) {
            wheel->integ -= CHASSIS_SPEED_KI * error * dt_s;
        }
    } else {
        /* 未饱和，正常 */
    }

    /* 静摩擦补偿（2026-11-18 修正，审计 ③）：
     * 占空比太小电机根本不转，PI 会误以为「还没动」而把积分顶满，
     * 所以要补一个最小启动占空比。
     *
     * ⚠️ 但只在「目标本身就不小于这个最小值」时才补：
     * 以前是只要 duty 非零就一律补到 70，于是
     *   forward=0、steer=40‰（偏角约 10 度）-> 两轮各 40 -> 被抬成 ±70，
     * 转向速率直接涨到 1.75 倍，表现为「小偏角一上来就猛」。
     * 现在小指令按原值给（轮子可能转不动，但那是真实的最小可动速度），
     * 补偿只负责「打破静摩擦」，不再放大指令。 */
    {
        const float target_mag = (target >= 0.0f) ? target : -target;

        if (target_mag >= (float)CHASSIS_SPEED_MIN_DUTY) {
            if ((wheel->duty > 0.0f) && (wheel->duty < (float)CHASSIS_SPEED_MIN_DUTY)) {
                wheel->duty = (float)CHASSIS_SPEED_MIN_DUTY;
            } else if ((wheel->duty < 0.0f) && (wheel->duty > -(float)CHASSIS_SPEED_MIN_DUTY)) {
                wheel->duty = -(float)CHASSIS_SPEED_MIN_DUTY;
            } else {
                /* 已经在死区外，不需要补 */
            }
        }
    }

    /* 速率限制：夹住「这一窗口相对上一窗口」的变化量。
     * 没有它的时候，PI 可以这一窗口给 +700、下一窗口给 -700，
     * 电机被瞬间反接 —— 车会猛地一顿，齿轮也跟着挨打。
     * 从 0 到 700‰ 需要 700/150 ≈ 5 个窗口 ≈ 100 ms，仍然比人手快。
     *
     * 按真实 dt 缩放：窗口被拉长到 40 ms 时允许的变化量也翻倍，
     * 否则「窗口更长、需要修得更多」时反而被卡住，表现为车顿一下。 */
    {
        const float slew = CHASSIS_SPEED_DUTY_SLEW * (dt_s / SPEED_LOOP_NOMINAL_DT_S);

        if ((wheel->duty - prev_duty) > slew) {
            wheel->duty = prev_duty + slew;
        } else if ((prev_duty - wheel->duty) > slew) {
            wheel->duty = prev_duty - slew;
        } else {
            /* 变化量在允许范围内 */
        }
    }

    wheel->duty = SpeedLoop_ClampF(wheel->duty, -duty_max, duty_max);
}

void SpeedLoop_Init(void)
{
    SpeedLoop_Reset();
}

void SpeedLoop_Reset(void)
{
    s_left.integ = 0.0f;
    s_left.meas = 0.0f;
    s_left.duty = 0.0f;
    s_right.integ = 0.0f;
    s_right.meas = 0.0f;
    s_right.duty = 0.0f;

    s_acc_left = 0;
    s_acc_right = 0;
    s_div_count = 0U;
    s_last_pi_ms = Delay_GetMs();

    /* 丢掉停车期间攒下的增量：否则下次起步第一拍会读到一个巨大的假速度，
     * 「速度突变」会让 PI 直接给出反向输出，车会先顿一下。 */
    (void)BSP_Encoder_GetLeftDelta();
    (void)BSP_Encoder_GetRightDelta();
}

void SpeedLoop_Update(int16_t target_left_per_mille, int16_t target_right_per_mille,
                      int16_t *duty_left_per_mille, int16_t *duty_right_per_mille)
{
    float target_left;
    float target_right;

    /* 每个控制拍都要把增量取走，不能跳拍，否则计数会被下一次一起算进来 */
    s_acc_left += (int32_t)BSP_Encoder_GetLeftDelta();
    s_acc_right += (int32_t)BSP_Encoder_GetRightDelta();

    s_div_count++;
    if (s_div_count < (uint8_t)CHASSIS_SPEED_LOOP_DIV) {
        /* 非 PI 拍：沿用上一次的结果，让占空比保持不动 */
        if (duty_left_per_mille != NULL) {
            *duty_left_per_mille = SpeedLoop_ToI16(s_left.duty);
        }
        if (duty_right_per_mille != NULL) {
            *duty_right_per_mille = SpeedLoop_ToI16(s_right.duty);
        }
        return;
    }

    s_div_count = 0U;

    /* 真实 dt：从上一个 PI 窗口到现在实际过了多久（ms）。
     * 主循环里 OLED 刷屏、sprintf 都会把某一拍拉长，用名义的 20 ms 会把
     * 这段时间内的计数当成「20 ms 走完的」——测速直接偏高 1.5~2 倍，
     * PI 于是以为车已经跑快了，把占空比砍下去；下一窗口又恢复正常……
     * 结果就是车「忽快忽慢」。夹在 5~100 ms 之间，防止计时器异常。 */
    {
        uint32_t now_ms = Delay_GetMs();
        uint32_t elapsed_ms = (uint32_t)(now_ms - s_last_pi_ms);
        float dt_s = (float)elapsed_ms / 1000.0f;

        if (dt_s < 0.005f) {
            dt_s = 0.005f;
        } else if (dt_s > 0.1f) {
            dt_s = 0.1f;
        }

        s_last_pi_ms = now_ms;

        /* 编码器 -> 速度。
         * 符号：本车「左轮前进时计数为正、右轮前进时计数为负」，而我们的速度
         * 约定是「正 = 后退」，所以左轮取反、右轮取正。这两个符号是实机测出来
         * 的（C-04），换电机/换接线后要重新确认。 */
        {
            const float speed_per_count = SPEED_PER_COUNT_AT(dt_s);

            s_left.meas = (float)s_acc_left * (float)CHASSIS_SPEED_SIGN_LEFT * speed_per_count;
            s_right.meas = (float)s_acc_right * (float)CHASSIS_SPEED_SIGN_RIGHT * speed_per_count;
        }

        /* 合理性限制：编码器被打到 / 线虚接时可能读出离谱的计数，
         * 直接当成「轮子在飞」，PI 会立刻反向打满。超限就夹住。 */
        s_left.meas = SpeedLoop_ClampF(s_left.meas,
                                       -CHASSIS_SPEED_MEAS_MAX, CHASSIS_SPEED_MEAS_MAX);
        s_right.meas = SpeedLoop_ClampF(s_right.meas,
                                        -CHASSIS_SPEED_MEAS_MAX, CHASSIS_SPEED_MEAS_MAX);

        s_acc_left = 0;
        s_acc_right = 0;

        target_left = (float)target_left_per_mille;
        target_right = (float)target_right_per_mille;

        SpeedLoop_Step(&s_left, target_left, dt_s);
        SpeedLoop_Step(&s_right, target_right, dt_s);
    }

    if (duty_left_per_mille != NULL) {
        *duty_left_per_mille = SpeedLoop_ToI16(s_left.duty);
    }
    if (duty_right_per_mille != NULL) {
        *duty_right_per_mille = SpeedLoop_ToI16(s_right.duty);
    }
}

int16_t SpeedLoop_GetLeftPermille(void)
{
    return SpeedLoop_ToI16(s_left.meas);
}

int16_t SpeedLoop_GetRightPermille(void)
{
    return SpeedLoop_ToI16(s_right.meas);
}
