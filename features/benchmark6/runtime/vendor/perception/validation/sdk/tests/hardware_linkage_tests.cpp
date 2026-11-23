#include "perception/types.hpp"
#include <iostream>
#include <stdexcept>

int main() {
    try {
        perception::Tensor tensor;
        tensor.channels=3;tensor.height=2;tensor.width=2;
        tensor.data.assign(12,0.f);
        tensor.validate();
        std::cout << "PASS: hardware_routes transitive dependency resolves Tensor::validate\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
