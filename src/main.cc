/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#include "cli/application.h"

int main(int argc, char **argv) {
  return amber::cli::Application(argc, argv).run();
}
