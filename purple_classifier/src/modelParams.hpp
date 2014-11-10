#ifndef MODELPARAMS_HPP
#define MODELPARAMS_HPP

#include <string>

class ModelParams {
public:
    ModelParams(){};
    ModelParams(std::string name, float mu_r, float mu_g, float mu_b, float std_r, float std_g, float std_b) :
        modelName(name), mu_r(mu_r), mu_g(mu_g), mu_b(mu_b), std_r(std_r),std_g(std_g),std_b(std_b){ }

    std::string modelName;
    float mu_r;
    float mu_g;
    float mu_b;
    float std_r;
    float std_g;
    float std_b;
};

#endif // MODELPARAMS_HPP
