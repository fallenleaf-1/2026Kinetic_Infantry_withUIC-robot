/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "memory.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
    // @todo: 不建议这样做,可扩展性差,不知道的开发者可能会误以为pid和config是同一个结构体
    // 后续修改为逐个赋值
    memset(pid, 0, sizeof(PIDInstance));
    // utilize the quality of struct that its memeory is continuous
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    // set rest of memory to 0
    pid->First_Calc = 1; // 首次计算跳过微分项(F6)
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

    // dt异常保护(F8): 微分/积分都要用dt,dt=0(或非有限)会直接产生inf/NaN;
    // 而NaN一旦被累加进Iout就会永久锁死(死区分支只清Output/ITerm,不清Iout),
    // 该电机此后永远拿不到力矩(实测: GM6020速度环 Iout=nan 导致四个舵轮全部失效)
    if (!isfinite(pid->dt) || pid->dt <= 0.0f)
        pid->dt = 1e-3f;
    // 状态污染保护(F8): 上一拍若混入非有限值,清掉积分/微分状态,本拍重新起算
    if (!isfinite(pid->Iout) || !isfinite(pid->Dout) || !isfinite(pid->Last_Err) || !isfinite(pid->Last_Measure))
    {
        pid->Iout = 0.0f;
        pid->Dout = 0.0f;
        pid->Last_Err = 0.0f;
        pid->Last_Measure = 0.0f;
        pid->Last_Dout = 0.0f;
    }
    // 反馈/参考本身异常时不下发输出,并清状态,避免异常值经积分器传播到下游
    if (!isfinite(measure) || !isfinite(ref))
    {
        pid->Measure = 0.0f;
        pid->Ref = ref;
        pid->Err = 0.0f;
        pid->Output = 0.0f;
        pid->ITerm = 0.0f;
        pid->Iout = 0.0f;
        pid->Dout = 0.0f;
        pid->First_Calc = 1; // 下一拍重新跳过微分,不用异常基准算微分
        return 0.0f;
    }

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 如果在死区外,则计算PID
    if (abs(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        if (pid->First_Calc)
        { // 首拍跳过微分,避免Last_Err/Last_Measure=0基准造成的D项冲击(F6)
            pid->Dout = 0;
        }
        else
        {
            pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        }

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行(首拍跳过,基准无效)
        if (!pid->First_Calc && (pid->Improve & PID_Derivative_On_Measurement))
            f_Derivative_On_Measurement(pid);
        // 微分滤波器(首拍跳过,基准无效)
        if (!pid->First_Calc && (pid->Improve & PID_DerivativeFilter))
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);

        pid->First_Calc = 0; // 首次有效计算完成,后续恢复微分
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
    }

    // 保存当前数据,用于下次计算
    // 输出仍是非有限值时(理论上已被上面的保护拦住)兜底清零,保证下游拿到的一定是有限值
    if (!isfinite(pid->Output))
    {
        pid->Output = 0.0f;
        pid->Iout = 0.0f;
        pid->Dout = 0.0f;
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

void PIDReset(PIDInstance *pid)
{ // 复位计算区,保留配置区(Kp..Derivative_LPF_RC);Stop/离线/模式切换时调用,防积存释放窜车(F7)
    pid->Measure = 0;
    pid->Last_Measure = 0;
    pid->Err = 0;
    pid->Last_Err = 0;
    pid->Last_ITerm = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->ITerm = 0;
    pid->Output = 0;
    pid->Last_Output = 0;
    pid->Last_Dout = 0;
    pid->Ref = 0;
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    pid->First_Calc = 1;          // 复位后首拍同样跳过微分
    DWT_GetDeltaT(&pid->DWT_CNT); // 重新锚定时间基准,避免复位后首个dt异常放大D项
}