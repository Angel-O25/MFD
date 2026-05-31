// This file includes ONLY model_severity.h.
#include "model_severity.h"
#include <Arduino.h>

int predictSeverity(float* features) {
    Eloquent::ML::Port::RandomForest rf;
    return rf.predict(features);
}