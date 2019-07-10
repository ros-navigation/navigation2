/*
 * gtest_main.cpp
 *
 *  Created on: Apr 6, 2012
 *      Author: tkruse
 */

#include <iostream>

#include <gtest/gtest.h>

int main(int argc, char **argv) {
  std::cout << "Running main() from gtest_main.cc\n";

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

