#include <random>
#include <cmath>

class NoiseGenerator {
public:
    NoiseGenerator() : gen(rd()), dist(0.0, 1.0) {}
    double gaussian(double mean, double std) { return mean + std * dist(gen); }
    double uniform(double low, double high) { return low + (high - low) * dist(gen); }  // Reuse for simple rand
private:
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> dist;
};

extern NoiseGenerator noise_gen;  // Global for ease


