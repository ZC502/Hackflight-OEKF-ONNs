#include <random>
#include <cmath>

struct NoiseParams {
    double sigma_gyro = 0.05;  // rad/s
    double sigma_accel = 0.1;  // m/s²
    double sigma_gps = 1.0;    // m
    double gps_jump_prob = 0.01;
};

class NoiseGen {
public:
    NoiseGen() : gen(rd()), gauss(0,1), uni(0,1) {}
    double gaussian(double mean, double std) { return mean + std * gauss(gen); }
    double uniform(double low, double high) { return low + (high-low) * uni(gen); }
private:
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> gauss;
    std::uniform_real_distribution<double> uni;
};

extern NoiseGen noise_gen;
extern NoiseParams noise_params;  // Global config
