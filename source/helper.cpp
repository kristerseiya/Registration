
#include "helper.h"

#include <random>

int UniformRandInt(const int min, const int max) {
    static thread_local std::mt19937 generator(std::random_device{}());
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(generator);
}
