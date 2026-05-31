// This file includes ONLY model_condition.h.
// Keeping it in its own .cpp means it compiles in a separate
// translation unit — no clash with model_severity.h.
#include "model_condition.h"
#include <Arduino.h>

int predictCondition(float* features) {
    Eloquent::ML::Port::RandomForest rf;
    return rf.predict(features);
}