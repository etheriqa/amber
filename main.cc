#include "cli/application.h"

int main(int argc, char **argv) {
  return amber::cli::Application<double>(argc, argv).run();
}
