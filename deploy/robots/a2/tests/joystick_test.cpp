#include "unitree_joystick_dsl.hpp"
#include <iostream>

int main() {
    using namespace unitree::common;
    UnitreeJoystick joy;
    auto predicate = [](const std::string& expression) {
        dsl::Parser parser(expression);
        return dsl::Compile(*parser.Parse());
    };
    auto stand = predicate("LT + up.on_pressed");
    auto velocity = predicate("RT + A.on_pressed");
    auto stop = predicate("LT + B.on_pressed");
    auto check = [](bool value) { if (!value) throw std::runtime_error("Joystick transition mismatch"); };
    check(!stand(joy) && !velocity(joy) && !stop(joy));
    joy.LT.pressed = true;
    joy.up.on_pressed = true;
    check(stand(joy) && !velocity(joy) && !stop(joy));
    joy.up.on_pressed = false;
    joy.B.on_pressed = true;
    check(stop(joy) && !stand(joy));
    joy.RT.pressed = true;
    joy.A.on_pressed = true;
    check(velocity(joy));
    if constexpr (!dsl::HasPressedTime<decltype(joy.LT)>::value) {
        bool rejected = false;
        try { predicate("LT(2s)"); } catch (const std::runtime_error&) { rejected = true; }
        check(rejected);
    }
    std::cout << "PASS: existing A2 joystick transitions\n";
}
