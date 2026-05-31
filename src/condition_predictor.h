#pragma once
// Wraps model_condition.h to isolate its RandomForest class
// from model_severity.h's identical class name.
int predictCondition(float* features);