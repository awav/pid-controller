#include "PID.h"
#include <algorithm>
#include <iostream>

PID::PID() {}

PID::~PID() {}

PID::PID(double kp, double ki, double kd, double min_out, double max_out) {
  if (!std::isnan(min_)) {
    min_ = min_out;
  }
  if (!std::isnan(max_)) {
    max_ = max_out;
  }
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  p_term_ = 0;
  i_term_ = 0;
  d_term_ = 0;
}

double PID::bound_minmax(double value) const {
  return std::min(std::max(value, min_), max_);
}

void PID::UpdateError(double cte) {
  if (std::isnan(cte_last_)) {
    cte_last_ = cte;
  }

  if (cte_last_ > min_ && cte_last_ < max_) {
    i_term_ += ki_ * cte;
    i_term_ = bound_minmax(i_term_);
  }

  p_term_ = kp_ * cte;
  d_term_ = kd_ * (cte - cte_last_);

  pid_last_ = p_term_ + i_term_ + d_term_;
  pid_last_ = bound_minmax(pid_last_);

  cte_last_ = cte;
}

double PID::TotalError() { return pid_last_; }

/*
bool PID::Tune() {
  // check ready for new input
  unsigned long now = millis();

  if (state == AUTOTUNER_OFF) {
    // initialize working variables the first time around
    peakType = NOT_A_PEAK;
    inputCount = 0;
    peakCount = 0;
    setpoint = *input;
    outputStart = *output;
    lastPeakTime[0] = now;
    workingNoiseBand = noiseBand;
    newWorkingNoiseBand = noiseBand;
    workingOstep = oStep;

    // move to new state
    state = RELAY_STEP_UP;
  }

  // otherwise check ready for new input
  else if ((now - lastTime) < sampleTime) {
    return false;
  }

  // get new input
  lastTime = now;
  double refVal = *input;

  // local flag variable
  bool justChanged = false;

  // check input and change relay state if necessary
  if ((state == RELAY_STEP_UP) && (refVal > setpoint + workingNoiseBand)) {
    state = RELAY_STEP_DOWN;
    justChanged = true;
  } else if ((state == RELAY_STEP_DOWN) &&
             (refVal < setpoint - workingNoiseBand)) {
    state = RELAY_STEP_UP;
    justChanged = true;
  }
  if (justChanged) {
    workingNoiseBand = newWorkingNoiseBand;
  } // if justChanged
// set output // FIXME need to respect output limits
  // not knowing output limits is one reason
  // to pass entire PID object to autotune method(s)
  if (((byte)state & (STEADY_STATE_AFTER_STEP_UP | RELAY_STEP_UP)) > 0) {
    *output = outputStart + workingOstep;
  } else if (state == RELAY_STEP_DOWN) {
    *output = outputStart - workingOstep;
  }

  // store initial inputs
  // we don't want to trust the maxes or mins
  // until the input array is full
  inputCount++;
  if (inputCount <= nLookBack) {
    lastInputs[nLookBack - inputCount] = refVal;
    return false;
  }

  // shift array of process values and identify peaks
  inputCount = nLookBack;
  bool isMax = true;
  bool isMin = true;
  for (int i = inputCount - 1; i >= 0; i--) {
    double val = lastInputs[i];
    if (isMax) {
      isMax = (refVal >= val);
    }
    if (isMin) {
      isMin = (refVal <= val);
    }
    lastInputs[i + 1] = val;
  }
  lastInputs[0] = refVal;

  // for AMIGOf tuning rule, perform an initial
  // step change to calculate process gain K_process
  // this may be very slow for lag-dominated processes
  // and may never terminate for integrating processes
  if (((byte)state & (STEADY_STATE_AT_BASELINE | STEADY_STATE_AFTER_STEP_UP)) >
      0) {
    // check that all the recent inputs are
    // equal give or take expected noise
    double iMax = lastInputs[0];
    double iMin = lastInputs[0];
    double avgInput = 0.0;
    for (byte i = 0; i <= inputCount; i++) {
      double val = lastInputs[i];
      if (iMax < val) {
        iMax = val;
      }
      if (iMin > val) {
        iMin = val;
      }
      avgInput += val;
    }
    avgInput /= (double)(inputCount + 1);

    // if recent inputs are stable
    if ((iMax - iMin) <= 2.0 * workingNoiseBand) {

#if defined(AUTOTUNE_RELAY_BIAS)
      lastStepTime[0] = now;
#endif

      if (state == STEADY_STATE_AT_BASELINE) {
        state = STEADY_STATE_AFTER_STEP_UP;
        lastPeaks[0] = avgInput;
        inputCount = 0;
        return false;
      }
      // else state == STEADY_STATE_AFTER_STEP_UP
      // calculate process gain
      K_process = (avgInput - lastPeaks[0]) / workingOstep;

#if defined(AUTOTUNE_DEBUG) || defined(USE_SIMULATION)
      Serial.print(F("Process gain "));
      Serial.println(K_process);
#endif

      // bad estimate of process gain
      if (K_process < 1e-10) // zero
      {
        state = AUTOTUNER_OFF;
        return false;
      }
      state = RELAY_STEP_DOWN;

#if defined(AUTOTUNE_RELAY_BIAS)
      sumInputSinceLastStep[0] = 0.0;
#endif

      return false;
    } else {
      return false;
    }
  }

  // increment peak count
  // and record peak time
  // for both maxima and minima
  justChanged = false;
  if (isMax) {
    if (peakType == MINIMUM) {
      justChanged = true;
    }
    peakType = MAXIMUM;
  } else if (isMin) {
    if (peakType == MAXIMUM) {
      justChanged = true;
    }
    peakType = MINIMUM;
  }

  // update peak times and values
  if (justChanged) {
    peakCount++;

#if defined(AUTOTUNE_DEBUG) || defined(USE_SIMULATION)
    Serial.println(F("peakCount "));
    Serial.println(peakCount);
    Serial.println(F("peaks"));
    for (byte i = 0; i < (peakCount > 4 ? 5 : peakCount); i++) {
      Serial.println(lastPeaks[i]);
    }
#endif

    // shift peak time and peak value arrays
    for (byte i = (peakCount > 4 ? 4 : peakCount); i > 0; i--) {
      lastPeakTime[i] = lastPeakTime[i - 1];
      lastPeaks[i] = lastPeaks[i - 1];
    }
  }
  if (isMax || isMin) {
    lastPeakTime[0] = now;
    lastPeaks[0] = refVal;

#if defined(AUTOTUNE_DEBUG)
    Serial.println();
    Serial.println(F("peakCount "));
    Serial.println(peakCount);
    Serial.println(F("refVal "));
    Serial.println(refVal);
    Serial.print(F("peak type "));
    Serial.println(peakType);
    Serial.print(F("isMin "));
    Serial.println(isMin);
    Serial.print(F("isMax "));
    Serial.println(isMax);
    Serial.println();
    Serial.println(F("lastInputs:"));
    for (byte i = 0; i <= inputCount; i++) {
      Serial.println(lastInputs[i]);
    }
    Serial.println();
#endif
  }

  // check for convergence of induced oscillation
  // convergence of amplitude assessed on last 4 peaks (1.5 cycles)
  double inducedAmplitude = 0.0;
  double phaseLag;
  if (

#if defined(AUTOTUNE_RELAY_BIAS)
      (stepCount > 4) &&
#endif

      justChanged && (peakCount > 4)) {
    double absMax = lastPeaks[1];
    double absMin = lastPeaks[1];
    for (byte i = 2; i <= 4; i++) {
      double val = lastPeaks[i];
      inducedAmplitude += abs(val - lastPeaks[i - 1]);
      if (absMax < val) {
        absMax = val;
      }
      if (absMin > val) {
        absMin = val;
      }
    }
    inducedAmplitude /= 6.0;

#if defined(AUTOTUNE_DEBUG) || defined(USE_SIMULATION)
    Serial.print(F("amplitude "));
    Serial.println(inducedAmplitude);
    Serial.print(F("absMin "));
    Serial.println(absMin);
    Serial.print(F("absMax "));
    Serial.println(absMax);
    Serial.print(F("convergence criterion "));
    Serial.println((0.5 * (absMax - absMin) - inducedAmplitude) /
                   inducedAmplitude);
#endif

    // source for AMIGOf PI auto tuning method:
    // "Revisiting the Ziegler-Nichols tuning rules for PI control —
    //  Part II. The frequency response method."
    // T. Hägglund and K. J. Åström
    // Asian Journal of Control, Vol. 6, No. 4, pp. 469-482, December 2004
    // http://www.ajc.org.tw/pages/paper/6.4PD/AC0604-P469-FR0371.pdf
    if (controlType == AMIGOF_PI) {
      phaseLag = calculatePhaseLag(inducedAmplitude);
      // check that phase lag is within acceptable bounds, ideally between 120°
      // and 140° but 115° to 145° will just about do, and might converge
      // quicker
      if (abs(phaseLag - CONST_PI * 130.0 / 180.0) >
          (CONST_PI * 15.0 / 180.0)) {
        // phase lag outside the desired range
        // set noiseBand to new estimate
        // aiming for 135° = 0.75 * pi (radians)
        // sin(135°) = sqrt(2)/2
        // NB noiseBand = 0.5 * hysteresis
        newWorkingNoiseBand = 0.5 * inducedAmplitude * CONST_SQRT2_DIV_2;

        return false;
      }
    }

    // check convergence criterion for amplitude of induced oscillation
    if (((0.5 * (absMax - absMin) - inducedAmplitude) / inducedAmplitude) <
        AUTOTUNE_PEAK_AMPLITUDE_TOLERANCE) {
      state = CONVERGED;
    }
  }

  // if the autotune has not already converged
  // terminate after 10 cycles
  // or if too long between peaks
  // or if too long between relay steps
  if (((now - lastPeakTime[0]) >
       (unsigned long)(AUTOTUNE_MAX_WAIT_MINUTES * 60000)) ||
      (peakCount >= 20)) {
    state = FAILED;
  }

  if (((byte)state & (CONVERGED | FAILED)) == 0) {
    return false;
  }

  // autotune algorithm has terminated
  // reset autotuner variables
  *output = outputStart;

  if (state == FAILED) {
    // do not calculate gain parameters
    return true;
  }

  // finish up by calculating tuning parameters

  // calculate ultimate gain
  double Ku = 4.0 * workingOstep / (inducedAmplitude * CONST_PI);

  // calculate ultimate period in seconds
  double Pu = (double)0.5 *
              ((lastPeakTime[1] - lastPeakTime[3]) +
               (lastPeakTime[2] - lastPeakTime[4])) /
              1000.0;

  // calculate gain parameters using tuning rules
  // NB PID generally outperforms PI for lag-dominated processes

  Kp = Ku / (double)tuningRule[controlType].divisor(KP_DIVISOR);
  Ti = Pu / (double)tuningRule[controlType].divisor(TI_DIVISOR);
  Td = tuningRule[controlType].PI_controller()
           ? 0.0
           : Pu / (double)tuningRule[controlType].divisor(TD_DIVISOR);

  // converged
  return true;
}
//*/
