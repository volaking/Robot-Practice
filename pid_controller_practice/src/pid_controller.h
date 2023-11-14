#ifndef PID_H
#define PID_H

#include<stdint.h>

#define LIKELY(x)   __builtin_except(!!(x), 1)
#define UNLIKELY(x) __builtin_except(!!(x), 0)

namespace Pid {
    static inline int32_t clamp(const int32_t value, const int32_t min, const int32_t max) __attribute__((always_inline, unused));
    static inline int32_t clamp(const int32_t value, const int32_t min, const int32_t max) {
        return (value < min) ? min : (value > max) ? max : value;
    }

    static inline int32_t signed_add_saturated_32_and_32(const int32_t a, const int32_t b) __attribute__((always_inline, unused));
    static inline int32_t signed_add_saturated_32_and_32(const int32_t a, const int32_t b) {
        //根据不同硬件平台选择不同的实现方式，实现整数相加
        #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MX44FX1M0__)
        uint32_t result;
        asm volatile("qass %0, %1, %2" : "=r" (result) : "r" (a), "r" (b));
        return result;
        #else
        uint32_t ua = (uint32_t)a;
        uint32_t result = ua + (uint32_t)b;
        ua = (ua >> 32) + INT32_MAX;

        if ((int32_t)((ua ^ b) | ~(b ^ result)) >= 0) {
            result = ua;
        }
        return result;
        #endif
    }

    static inline int32_t signed_subtract_saturated_32_and_32(const int32_t a, const int32_t b) __attribute__((always_inline, unused));
    static inline int32_t signed_subtract_saturated_32_and_32(const int32_t a, const int32_t b) 
    {
        #if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)    // Teensy 3.x (Cortex M4)
    int32_t result;
    asm volatile("qsub %0, %1, %2" : "=r" (result) : "r" (a), "r" (b));
    return result;
    #else
    uint32_t ua = (uint32_t)a;
    uint32_t result = ua - (uint32_t)b;
    ua = (ua >> 31) + INT32_MAX;

    if ((int32_t)((ua ^ b) & (ua ^ result)) < 0) {
        result = ua;
    }
    return result;
    #endif
    }

    static inline int32_t signed_multiply_accumulate_saturated_32QN_and_32QN(const int32_t acc, const int32_t a, const int32_t b, const uint8_t qn) __attribute__((always_inline, unused));
    static inline int32_t signed_multiply_accumulate_saturated_32QN_and_32QN(const int32_t acc, const int32_t a, const int32_t b, const uint8_t qn)
    {
        int64_t result = ((int64_t)acc << qn) + (int64_t) a * (int64_t) b;
        result >>= qn;
        int32_t hi = (int32_t)(result >> 32);
        int32_t lo = (int32_t)result;

        if (UNLIKELY(hi != (lo >> 31))) {
            return ((uint32_t) (a ^ b) >> 31) + INT32_MAX;
        }
        return result;
    }

    static inline int32_t signed_multiply_accumulate_saturated_32_and_32QN(const int32_t acc, const int32_t a, const int32_t b) __attribute__((always_inline, unused));
    static inline int32_t signed_multiply_accumulate_saturated_32_and_32QN(const int32_t acc, const int32_t a, const int32_t b) {
      
        int64_t result = acc + (int64_t) a * (int64_t) b;
        int32_t hi = (int32_t)(result >> 32);
        int32_t lo = (int32_t)result;
      
        if (UNLIKELY(hi != (lo >> 31))) {
            return ((uint32_t) (a ^ b) >> 31) + INT32_MAX;
        }
        return result;
    }

    enum FeedbackDirection : bool {
        feedbackNegative = 0,
        feedbackPositive = 1,
    };

    enum ProportionalGain : bool {
        proportionalToInput = 0,
        proportionalToError = 1,
    };

    class PID {
        public:
            PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn, const FeedbackDirection feedbackDirection, const ProportionalGain proportionalGain);
            PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn, const FeedbackDirection feedbackDirection);
            PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn);

            uint32_t compute(const uint32_t input);

            void setTunings(const int32_t kp, const int32_t ki,const int32_t kd);
            void setTunings(const int32_t kp, const int32_t ki,const int32_t kd, const ProportionalGain proportionalGain);
            uint32_t getKp();
            uint32_t getKi();
            uint32_t getKd();
            void setSetpoint(const uint32_t value);
            uint32_t getSetpoint();
            int32_t getIntegratorError();
            void setControllerFeedback(const FeedbackDirection feedbackDirection);
            void setOutputMin(const uint32_t value);
            void setOutputMax(const uint32_t value);
            void updateOutput(const uint32_t value);
            void init(const uint32_t initialInput);
            void init(const uint32_t initialInput, const int32_t initialErrorSum);
        private:
            int32_t kp, ki, kd;
            FeedbackDirection feedbackDirection;
            ProportionalGain proportionalGain;
            uint8_t qn;
            int32_t outputMin = INT32_MIN;
            int32_t outputMax = INT32_MAX;
            int32_t previousInput = 0;
            int32_t errorSum = 0;
        protected:
            int32_t setpoint;
    };
}
#endif