

#ifndef TEMPERATURE_MODEL_H
#define TEMPERATURE_MODEL_H

#define TEMP_MODEL_Q    6       // Q6 numbers

void init_temperature_model();
int step_temperature_model(int current_ma);
int get_model_temperature_cK();


#endif
