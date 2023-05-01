#include <string.h>

#include <string>
#include <vector>

#include "crl-basic/utils/mathDefs.h"

namespace locoApp {

struct RobotModelOption {
    enum class Type {
        BOB = 0,
        DOG = 1,
    };

    Type type;
    std::string name;
    std::string filePath;
    std::vector<std::pair<std::string, std::string>> legs;
    double baseTargetHeight;
    double swingFootHeight;
};

const std::vector<RobotModelOption> modelOptions = {
    {
        RobotModelOption::Type::BOB,                 //
        "Bob",                                       //
        CRL_DATA_FOLDER "/robots/bob/bob.rbs",  //
        {},
        0.31,  //
        0.07,  //
    },

    // unitree laikago from rbs
    {
        RobotModelOption::Type::DOG,                 //
        "Dog",                                       //
        CRL_DATA_FOLDER "/robots/dog/dog.rbs",  //
        {
            {"fl", "tibia_0"},  //
            {"hl", "tibia_1"},  //
            {"fr", "tibia_2"},  //
            {"hr", "tibia_3"},  //
        },
        0.437,  //
        0.1,    //
    },
};

}  // namespace locoApp
