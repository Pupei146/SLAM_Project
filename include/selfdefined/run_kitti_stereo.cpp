#include <gflags/gflags.h>
#include "visualOdometry.h"

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    selfdefined::VisualOdometry::Ptr vo(new selfdefined::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}