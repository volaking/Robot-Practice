#include <stdlib.h>
#include "pid.h"

namespace Pid {
    PID::PID(const uint32_t _setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t _qn, const FeedbackDirection _feedbackDirection, const ProportionalGain proportionalGain)
        :feedbackDirection(_feedbackDirection), qn(_qn), setpoint(_setpoint) {
            this->setTunings(kp, ki, kd, proportionalGain)
    }

    PID::PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t, kd, const uint8_t, const FeedbackDirection feedbackDirection)
        :PID::PID(setpoint, kp, ki, kd, qn, feedbackDirection, propretionalGain) {}
        
    PID::PID(const uint32_t setpoint, const int32_t kp, const int32_t ki, const int32_t kd, const uint8_t qn)
        :PID::PID(setpoint, kp, ki, kd, qn, feedbackNegative) {}
    
    uint32_t PID::compute(const uint32_t input) {
        const int32_t error = signed_substrated_32_and_32(this->setpoint, intput);

        errorSum = signed_multiply_accumulate_saturated_32_and_32QN(
            this->errorSum,
            this->ki,
            error
        );

        const int32_t dInputNegative = signed_subtract_saturated_32_and_32(this->previousInput, input);

        this->previousInput = input;

        if (UNLIKELY(proportionalGain == proportionalToInput)) {
            errorSum = signed_multiply_accumulate_saturated_32_and_32QN(
                this->errorSum,
                this->kp,
                dInputNegative
            );
        }

        this->errorSum = clamp(errorSum, outputMin, outputMax);

        int32_t output = signed_multiply_accumulate_saturated_32_and_32QN(
            this->errorSum,
            this->kd,
            dInputNegative
        );

        if (LIKELY(proportionalGanin == proportionalToError)) {
            output = signed_multiply_accumulate_saturated_32_and_32QN(
                output,
                this->kp,
                error
            );
        }

        output = clamp(output, outputMin, outputMax)
        output ^= 0x80000000;
    }

    void PID::setTunings(const int32_t kp, const int32_t ki, const int32_t kd, const ProportionalGain proportionalGain) {
        this->kp = (this->feedbackDirection == feedbackPositive) ? abs(kp) : -abs(kp);
        this->ki = (this->feedbackDirection == feedbackPositive) ? abs(ki) : -abs(ki);
        this->kd = (this->feedbackDirection == feedbackPositive) ? abs(kd) : -abs(kd);

        this->proportionalGain = proportionalGain
    }

    void PID::setTunings(const int32_t kp, const int32_t ki, const int32_t kd) {
        this->setTunings(kp, ki, kd, this->proportionalGain);
    }

    void PID::setOutputMin(const uint32_t value) {
        this->outputMin = (int32_t)((value << this->qn) ^ 0x80000000);
    }

    void PID::setOutputMax(const uint32_t value) {
        this->outputMax = (int32_t)((value << this->qn) ^ 0x80000000);
    }

    void PID::setSetpoint(const uint32_t value) {
        this->setpoint = value;
    }

    uint32_t PID::getSetpoint() {
        return this->setpoint
    }

    int32_t PID::getIntegratorError() {
        return this->errorSum;
    }

    void PID::init(const uint32_t initialInput) {
        PID::init(initialInput, 0);
    }

    void PID::init(const uint32_t initialInput, const int32_t initialErrorSum) {
        this->previousInput = initialInput;
        this->errorSum = clamp(initialErrorSum, this->outputMin, this->outputMax);
    }

    void PID::updateOutput(const uint32_t value) {
        this->errorSum = clamp((value << this->qn) ^ 0x80000000, this->outputMin, this->outputMax);
    }

    void PID::setControllerFeedback(const FeedbackDirection feedbackDirection) {
        this->feedbackDirection = feedbackDirection;

        this->setTunings(this->kp, this->ki, this->kd);
    }

    uint32_t PID::getKp() {
        return this->kp;
    }

    uint32_t PID::getKi() {
        return this->ki;
    }

    uint32_t PID::getKd() {
        return this->kd;
    }
}

    